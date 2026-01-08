// Program.cs
// dotnet add package MAVLink --version 1.0.8

using System;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;

internal static class Program
{
    // ---------------- CONFIG ----------------
    private const string HID_PATH =
        @"\\?\hid#vid_0483&pid_5710#6&200f1eb3&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";

    private const string MAVLINK_HOST = "192.168.144.11";
    private const int MAVLINK_PORT = 2000;

    private static bool modeInitialized = false;

    private const int PWM_LOW = 1100;
    private const int PWM_HIGH = 1900;

    private const byte TH1 = 0;
    private const byte TH2 = 127;
    private const byte TH3 = 129;

    private const int RC_RATE_MS = 50;     // 20 Hz
    private const int CTRL_RATE_MS = 100;  // 10 Hz
    private const int STABLE_COUNT = 5;

    // watchdog / mavlink flags
    private const int TCP_RETRY_MS = 1000;
    private const int MAVLINK_ALIVE_MS = 3000;   // heartbeat freshness => "mavlink connected"
    private const int SYS_COMP_CHECK_MS = 2000;  // check every 2 seconds

    // --- debounce state ---
    private static byte _stableButtons;
    private static int _buttonsCount = 0;

    private static string? _stableMode;
    private static int _modeCount = 0;

    private static bool _lastRtlButton = false;

    // Your axis calibration
    private static readonly AxisCfg[] AXES =
    {
        new AxisCfg("roll",     BufIndex:4,  Min:163, Center:30,  Max:127, RcIndex1Based:1),
        new AxisCfg("pitch",    BufIndex:5,  Min:160, Center:31,  Max:127, RcIndex1Based:2),
        new AxisCfg("throttle", BufIndex:8,  Min:129, Center:228, Max:99,  RcIndex1Based:3),
        new AxisCfg("yaw",      BufIndex:7,  Min:129, Center:0,   Max:127, RcIndex1Based:4),
    };

    private static readonly AxisCfg[] EXTRA =
    {
        new AxisCfg("rc10", BufIndex:9,  Min:127, Center:252, Max:129, RcIndex1Based:5),
        new AxisCfg("rc11", BufIndex:10, Min:127, Center:245, Max:129, RcIndex1Based:6),
    };

    // ---------- Win32 HID ----------
    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    private static extern IntPtr CreateFileW(
        string lpFileName,
        uint dwDesiredAccess,
        uint dwShareMode,
        IntPtr lpSecurityAttributes,
        uint dwCreationDisposition,
        uint dwFlagsAndAttributes,
        IntPtr hTemplateFile);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool ReadFile(
        IntPtr hFile,
        byte[] lpBuffer,
        uint nNumberOfBytesToRead,
        out uint lpNumberOfBytesRead,
        IntPtr lpOverlapped);

    private const uint GENERIC_READ = 0x80000000;
    private const uint OPEN_EXISTING = 3;
    private const uint FILE_SHARE_READ = 1;
    private const uint FILE_SHARE_WRITE = 2;
  


    // ---------- MAVLink ----------
    private static readonly MAVLink.MavlinkParse Parser = new MAVLink.MavlinkParse();

    // target (used for sending)
    private static byte _targetSys = 1;
    private static byte _targetComp = 1;

    // learned from heartbeat (drainer parses)
    private static byte _hbSys = 0;
    private static byte _hbComp = 0;
    private static long _lastHbMs = 0; // use Volatile

    // our sender identity
    private const byte OUR_SYS = 255;
    private const byte OUR_COMP = 190;

    // TCP shared
    private static readonly object _netLock = new();
    private static TcpClient? _tcp;
    private static NetworkStream? _stream;
    private static volatile bool _tcpUp = false;

    private static void Main()
    {
        // TCP watchdog (connect/reconnect only)
        new Thread(TcpWatchdog) { IsBackground = true }.Start();

        // Open HID (unchanged)
        IntPtr h = CreateFileW(
            HID_PATH,
            GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            IntPtr.Zero,
            OPEN_EXISTING,
            0,
            IntPtr.Zero);

        if (h == new IntPtr(-1))
            throw new Exception("FAILED TO OPEN HID (check path / permissions)");

        var buf = new byte[64];

        string? lastMode = null;
        bool rtlActive = false;

        var lastRc = new ushort[8];
        var rcTmp = new ushort[8];

        long nextRc = Environment.TickCount64;
        long nextCtrl = Environment.TickCount64;
        long nextSysCompCheck = Environment.TickCount64;

        while (true)
        {
            if (!ReadFile(h, buf, 64, out uint n, IntPtr.Zero)) continue;
            if (n < 12) continue;

            long now = Environment.TickCount64;

            // every 2 seconds: verify sys/comp matches what heartbeat says
            if (now >= nextSysCompCheck)
            {
                bool mavUp = (now - Volatile.Read(ref _lastHbMs)) <= MAVLINK_ALIVE_MS;
                if (mavUp && _hbSys != 0 && _hbComp != 0)
                {
                    if (_targetSys != _hbSys || _targetComp != _hbComp)
                    {
                        _targetSys = _hbSys;
                        _targetComp = _hbComp;
                        Console.WriteLine($"[OK] Target sys/comp corrected to {_targetSys}/{_targetComp}");
                    }
                }

                nextSysCompCheck = now + SYS_COMP_CHECK_MS;
            }

            NetworkStream? stream;
            lock (_netLock) stream = _stream;

            // ---------- RC OVERRIDE (20 Hz) ----------
            if (now >= nextRc)
            {
                Array.Clear(rcTmp, 0, rcTmp.Length);

                foreach (var a in AXES)
                {
                    byte raw = buf[a.BufIndex];
                    ushort pwm = AxisToPwm(raw, a.Min, a.Center, a.Max);
                    rcTmp[a.RcIndex1Based - 1] = pwm;
                }

                foreach (var a in EXTRA)
                {
                    byte raw = buf[a.BufIndex];
                    ushort pwm = AxisToPwmCircular(raw, a.Min, a.Center, a.Max);
                    rcTmp[a.RcIndex1Based - 1] = pwm;
                }

                if (stream != null) SendRcOverride(stream, rcTmp);
                Array.Copy(rcTmp, lastRc, 8);

                nextRc = now + RC_RATE_MS;
            }

            // ---------- BUTTONS / MODES (10 Hz) ----------
            if (now >= nextCtrl)
            {
                byte buttons = buf[1];
                byte modeRaw = buf[6];

                if (buttons == _stableButtons) _buttonsCount++;
                else { _stableButtons = buttons; _buttonsCount = 1; }

                if (_buttonsCount >= STABLE_COUNT)
                {
                    if (stream != null)
                    {
                        SendDoSetServo(stream, 10, (buttons & 1) != 0 ? PWM_HIGH : PWM_LOW);
                        SendDoSetServo(stream, 11, (buttons & 2) != 0 ? PWM_HIGH : PWM_LOW);
                        SendDoSetServo(stream, 12, (buttons & 4) != 0 ? PWM_HIGH : PWM_LOW);
                        SendDoSetServo(stream, 13, (buttons & 8) != 0 ? PWM_HIGH : PWM_LOW);
                    }

                    bool rtlButton = (buttons & 16) != 0;

                    if (rtlButton && !_lastRtlButton)
                    {
                        rtlActive = true;
                        if (stream != null) SetModeCopter(stream, "RTL");
                    }
                    else if (!rtlButton && _lastRtlButton)
                    {
                        rtlActive = false;
                        if (lastMode != null && stream != null) SetModeCopter(stream, lastMode);
                    }

                    _lastRtlButton = rtlButton;
                }

                string? m = DecodeMode(modeRaw);
                // ---- boot init: latch current mode stick immediately ----
                if (!modeInitialized && m != null)
                {
                    lastMode = m;
                    _stableMode = m;
                    _modeCount = STABLE_COUNT;  // treat as stable on boot
                    modeInitialized = true;
                }

                if (m == _stableMode) _modeCount++;
                else { _stableMode = m; _modeCount = 1; }

                if (_modeCount == STABLE_COUNT && !rtlActive && m != lastMode)
                {
                    if (m != null && stream != null) SetModeCopter(stream, m);
                    lastMode = m;
                }

                bool mavlinkUp = (now - Volatile.Read(ref _lastHbMs)) <= MAVLINK_ALIVE_MS;

                Console.WriteLine(
                    $"btn={buttons,3} modeRaw={modeRaw,3} | TCP={(_tcpUp ? "UP" : "DOWN")} MAV={(mavlinkUp ? "UP" : "WAIT")} sys/comp={_targetSys}/{_targetComp} | RTL={(rtlActive ? "ON" : "OFF")} MODE={lastMode}"
                );

                nextCtrl = now + CTRL_RATE_MS;
            }

            Thread.Sleep(5);
        }
    }

    // ----------------- TCP watchdog (simple) -----------------
    private static void TcpWatchdog()
    {
        while (true)
        {
            if (!_tcpUp)
            {
                try
                {
                    var tcp = new TcpClient();
 

                    tcp.Connect(MAVLINK_HOST, MAVLINK_PORT);
                    tcp.NoDelay = true;
                    var stream = tcp.GetStream();

                    tcp.Client.SendTimeout = 500;
                    tcp.Client.ReceiveTimeout = 500;
                    stream.WriteTimeout = 500;
                    stream.ReadTimeout = 500;

                    lock (_netLock)
                    {
                        try { _stream?.Close(); } catch { }
                        try { _tcp?.Close(); } catch { }
                        _tcp = tcp;
                        _stream = stream;
                    }

                    _tcpUp = true;
                    Console.WriteLine("[OK] TCP connected");

                    // single reader: drain + heartbeat detect
                    new Thread(() => RxDrainAndHeartbeat(stream)) { IsBackground = true }.Start();
                }
                catch
                {
                    _tcpUp = false;
                    Thread.Sleep(TCP_RETRY_MS);
                }
            }

            Thread.Sleep(200);
        }
    }

    private static void TcpDown()
    {
        _tcpUp = false;
        lock (_netLock)
        {
            try { _stream?.Close(); } catch { }
            try { _tcp?.Close(); } catch { }
            _stream = null;
            _tcp = null;
        }
    }

    private const int WRITE_TIMEOUT_MS = 500;

    private static void SafeWrite(NetworkStream stream, byte[] pkt)
    {
        try
        {
            // Never let the main thread hang on a TCP write.
            var t = stream.WriteAsync(pkt, 0, pkt.Length);
            if (!t.Wait(WRITE_TIMEOUT_MS))
            {
                TcpDown();
            }
        }
        catch
        {
            TcpDown();
        }
    }

    // ----------------- MODE -----------------
    private static string? DecodeMode(byte v)
    {
        if (v == TH1) return "AUTO";
        if (v == TH2) return "LOITER";
        if (v == TH3) return "ALT_HOLD";
        return null;
    }

    private static uint CopterCustomMode(string name) => name switch
    {
        "ALT_HOLD" => 2u,
        "AUTO" => 3u,
        "LOITER" => 5u,
        "RTL" => 6u,
        _ => 0u
    };

    private static void SetModeCopter(NetworkStream stream, string modeName)
    {
        const uint MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1u;
        uint custom = CopterCustomMode(modeName);
        if (custom == 0) return;

        SendCommandLong(stream, 176, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom, 0, 0, 0, 0, 0);
    }

    // ----------------- AXIS MAP -----------------
    private static int Unwrap(int v, int c)
    {
        int d = v - c;
        if (d > 128) d -= 256;
        else if (d < -128) d += 256;
        return c + d;
    }

    private static ushort AxisToPwm(byte v, int amin, int ac, int amax)
    {
        int vv = Unwrap(v, ac);
        int mn = Unwrap(amin, ac);
        int mx = Unwrap(amax, ac);

        if (vv >= ac)
            return (ushort)Math.Clamp((int)(1500 + (vv - ac) * 500.0 / (mx - ac)), 1000, 2000);
        return (ushort)Math.Clamp((int)(1500 - (ac - vv) * 500.0 / (ac - mn)), 1000, 2000);
    }

    private static ushort AxisToPwmCircular(byte v, int amin, int ac, int amax)
    {
        static int Norm(int x) => ((x % 256) + 256) % 256;
        static int Dist(int a, int b) => (b - a) & 0xFF;

        int vv = Norm(v);
        int mn = Norm(amin);
        int cc = Norm(ac);
        int mx = Norm(amax);

        int spanDown = Dist(cc, mn);
        int spanUp = Dist(cc, mx);
        int valDown = Dist(cc, vv);
        int valUp = Dist(vv, cc);

        if (valDown <= spanDown && spanDown != 0)
            return (ushort)Math.Clamp((int)(1500 - valDown * 500.0 / spanDown), 1000, 2000);

        if (spanUp != 0)
            return (ushort)Math.Clamp((int)(1500 + valUp * 500.0 / spanUp), 1000, 2000);

        return 1500;
    }

    // ----------------- MAVLink send helpers -----------------
    private static void SendRcOverride(NetworkStream stream, ushort[] rc8)
    {
        var msg = new MAVLink.mavlink_rc_channels_override_t
        {
            target_system = _targetSys,
            target_component = _targetComp,
            chan1_raw = rc8[0],
            chan2_raw = rc8[1],
            chan3_raw = rc8[2],
            chan4_raw = rc8[3],
            chan5_raw = rc8[4],
            chan6_raw = rc8[5],
            chan7_raw = 0,
            chan8_raw = 0
        };

        byte[] pkt = Parser.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_OVERRIDE, msg, OUR_SYS, OUR_COMP);
        try { SafeWrite(stream, pkt); }
        catch { TcpDown(); }
    }

    private static void SendDoSetServo(NetworkStream stream, int servo, int pwm)
    {
        SendCommandLong(stream, 183, servo, pwm, 0, 0, 0, 0, 0);
    }

    private static void SendCommandLong(
        NetworkStream stream,
        int command,
        double p1, double p2, double p3, double p4, double p5, double p6, double p7)
    {
        var msg = new MAVLink.mavlink_command_long_t
        {
            target_system = _targetSys,
            target_component = _targetComp,
            command = (ushort)command,
            confirmation = 0,
            param1 = (float)p1,
            param2 = (float)p2,
            param3 = (float)p3,
            param4 = (float)p4,
            param5 = (float)p5,
            param6 = (float)p6,
            param7 = (float)p7
        };

        byte[] pkt = Parser.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, msg, OUR_SYS, OUR_COMP);
        try { SafeWrite(stream, pkt); }
        catch { TcpDown(); }
    }

    // ----------------- RX drain + heartbeat detect (single reader) -----------------
    
    private static void RxDrainAndHeartbeat(NetworkStream stream)
    {
        var b = new byte[4096];

        // stability filter
        byte lastSys = 0, lastComp = 0;
        int sameCount = 0;

        // parser state
        int st = 0;
        bool v2 = false;
        int len = 0, payloadLeft = 0, sigLeft = 0;
        byte sys = 0, comp = 0;
        uint msgid = 0;
        byte v2Incompat = 0;

        void SeenHeartbeat(byte s, byte c)
        {
            if (s == 0 || c == 0) return;

            if (s == lastSys && c == lastComp) sameCount++;
            else { lastSys = s; lastComp = c; sameCount = 1; }

            if (sameCount >= 3)
            {
                _hbSys = s;
                _hbComp = c;
                Volatile.Write(ref _lastHbMs, Environment.TickCount64);
            }
        }

        while (_tcpUp)
        {
            try
            {
                int n = stream.Read(b, 0, b.Length);
                if (n <= 0) { TcpDown(); return; }

                for (int i = 0; i < n; i++)
                {
                    byte c = b[i];

                    switch (st)
                    {
                        case 0:
                            if (c == 0xFE) { v2 = false; st = 1; }
                            else if (c == 0xFD) { v2 = true; st = 1; }
                            break;

                        case 1:
                            len = c;
                            msgid = 0;
                            sys = comp = 0;
                            v2Incompat = 0;
                            st = v2 ? 2 : 5;
                            break;

                        case 2: v2Incompat = c; st = 3; break; // incompat
                        case 3: st = 4; break;                 // compat
                        case 4: st = 5; break;                 // seq

                        case 5: sys = c; st = 6; break;
                        case 6: comp = c; st = v2 ? 7 : 10; break;

                        case 7: msgid = c; st = 8; break;
                        case 8: msgid |= (uint)c << 8; st = 9; break;
                        case 9:
                            msgid |= (uint)c << 16;
                            payloadLeft = len;
                            st = (payloadLeft == 0) ? 11 : 10;
                            break;

                        case 10:
                            payloadLeft--;
                            if (payloadLeft <= 0) st = 11;
                            break;

                        case 11: st = 12; break; // CRC1
                        case 12:
                            // CRC2 -> frame complete (no CRC validation)
                            if (msgid == 0) SeenHeartbeat(sys, comp);
                            sigLeft = (v2 && (v2Incompat & 0x01) != 0) ? 13 : 0;
                            st = (sigLeft > 0) ? 13 : 0;
                            break;

                        case 13:
                            sigLeft--;
                            if (sigLeft <= 0) st = 0;
                            break;
                    }
                }
            }
            catch (IOException ex) when (ex.InnerException is SocketException se && se.SocketErrorCode == SocketError.TimedOut)
            {
                continue; // no data right now
            }
            catch
            {
                TcpDown();
                return;
            }
        }
    }


    private readonly record struct AxisCfg(string Name, int BufIndex, int Min, int Center, int Max, int RcIndex1Based);
}
