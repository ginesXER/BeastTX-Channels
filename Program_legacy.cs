// Program.cs
// dotnet add package MAVLink --version 1.0.8

using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;

internal static class Program_legacy
{
    // ---------------- CONFIG ----------------
    private const string HID_PATH =
        @"\\?\hid#vid_0483&pid_5710#6&200f1eb3&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";

    private const string MAVLINK_HOST = "192.168.144.11";
    private const int MAVLINK_PORT = 2000;

    private const int PWM_LOW = 1100;
    private const int PWM_HIGH = 1900;

    private const byte TH1 = 0;
    private const byte TH2 = 127;
    private const byte TH3 = 129;


    private const int RC_RATE_MS = 50;   // 20 Hz
    private const int CTRL_RATE_MS = 100; // 10 Hz
    private const int STABLE_COUNT = 5;

    // --- debounce state ---
    private static byte _stableButtons;
    private static int _buttonsCount = 0;

    private static string? _stableMode;
    private static int _modeCount = 0;

    private static bool _stableRtl;
    private static int _rtlCount = 0;

    private static bool _lastRtlButton = false;

    // Your axis calibration (same meaning as python)
    private static readonly AxisCfg[] AXES =
    {
        new AxisCfg("roll",     BufIndex:4,  Min:163, Center:30,  Max:127, RcIndex1Based:1),
        new AxisCfg("pitch",    BufIndex:5,  Min:160, Center:31,  Max:127, RcIndex1Based:2),
        new AxisCfg("throttle", BufIndex:8,  Min:129, Center:228, Max:99,  RcIndex1Based:3),
        new AxisCfg("yaw",      BufIndex:7,  Min:129, Center:0,   Max:127, RcIndex1Based:4),
    };

    private static readonly AxisCfg[] EXTRA =
    {
        // your “circular” knobs
        new AxisCfg("rc10", BufIndex:9,  Min:127, Center:252, Max:129, RcIndex1Based:5),
        new AxisCfg("rc11", BufIndex:10, Min:127, Center:245, Max:129, RcIndex1Based:6),
    };
    // ----------------------------------------

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

    // You can hard-set these if you want, but we’ll learn them from HEARTBEAT
    private static byte _targetSys = 1;
    private static byte _targetComp = 1;

    // our sender identity
    private const byte OUR_SYS = 255;
    private const byte OUR_COMP = 190;

    private static volatile bool _rxDrainRun = true;

    private static void Main_old()
    {
        using var tcp = new TcpClient();
        tcp.Connect(MAVLINK_HOST, MAVLINK_PORT);
        tcp.NoDelay = true;

        using NetworkStream stream = tcp.GetStream();
        Console.WriteLine("[OK] TCP connected");

        // Drain RX so server buffers never fill (same idea as your python)
        var rxThread = new Thread(() => RxDrain(stream)) { IsBackground = true };
        rxThread.Start();

        // Wait heartbeat to learn sysid/compid (best effort)
        WaitHeartbeat(stream, timeoutMs: 5000);
        Console.WriteLine($"[OK] Target sys/comp = {_targetSys}/{_targetComp}");

        // Open HID
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

        // state
        byte? lastButtons = null;
        string? lastMode = null;
        bool rtlActive = false;

        var lastRc = new ushort[8]; // 0 means "no override"
        var rcTmp = new ushort[8];

        long nextRc = Environment.TickCount64;
        long nextCtrl = Environment.TickCount64;

        while (true)
        {
            if (!ReadFile(h, buf, 64, out uint n, IntPtr.Zero)) continue;
            if (n < 12) continue;

            long now = Environment.TickCount64;

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

                if (!Same(rcTmp, lastRc))
                {
                    SendRcOverride(stream, rcTmp);
                    Array.Copy(rcTmp, lastRc, 8);
                }

                nextRc = now + RC_RATE_MS;
            }

            // ---------- BUTTONS / MODES (10 Hz) ----------
            if (now >= nextCtrl)
            {
                byte buttons = buf[1];
                byte modeRaw = buf[6];
                // ---- debounce buttons ----
                if (buttons == _stableButtons)
                {
                    _buttonsCount++;
                }
                else
                {
                    _stableButtons = buttons;
                    _buttonsCount = 1;
                }

                if (_buttonsCount >= STABLE_COUNT)
                {
                    // servos 10..13
                    SendDoSetServo(stream, 10, (buttons & 1) != 0 ? PWM_HIGH : PWM_LOW);
                    SendDoSetServo(stream, 11, (buttons & 2) != 0 ? PWM_HIGH : PWM_LOW);
                    SendDoSetServo(stream, 12, (buttons & 4) != 0 ? PWM_HIGH : PWM_LOW);
                    SendDoSetServo(stream, 13, (buttons & 8) != 0 ? PWM_HIGH : PWM_LOW);

                    bool rtlButton = (buttons & 16) != 0;

                    // ---- RTL EDGE DETECT ----
                    if (rtlButton && !_lastRtlButton)
                    {
                        // rising edge: OFF -> ON
                        rtlActive = true;
                        SetModeCopter(stream, "RTL");
                    }
                    else if (!rtlButton && _lastRtlButton)
                    {
                        // falling edge: ON -> OFF
                        rtlActive = false;
                        if (lastMode != null)
                        SetModeCopter(stream, lastMode);
                    }

                    _lastRtlButton = rtlButton;
                }




                string m = DecodeMode(modeRaw);

                // ---- debounce mode ----
                if (m == _stableMode)
                {
                    _modeCount++;
                }
                else
                {
                    _stableMode = m;
                    _modeCount = 1;
                }

                if (_modeCount == STABLE_COUNT && !rtlActive && m != lastMode)
                {
                    SetModeCopter(stream, m);
                    lastMode = m;
                }


                Console.WriteLine(
                    $"btn={buttons,3} modeRaw={modeRaw,3} | RTL={(rtlActive ? "ON" : "OFF")} MODE={lastMode}");

                nextCtrl = now + CTRL_RATE_MS;
            }

            Thread.Sleep(5);
        }
    }

    // ----------------- MODE -----------------
    private static string DecodeMode(byte v)
    {
        if (v == TH1) return "AUTO";
        if (v == TH2) return "LOITER";
        if (v == TH3) return "ALT_HOLD";
        else return null;
    }

    // ArduCopter custom_mode values (common defaults):
    // ALT_HOLD=2, AUTO=3, LOITER=5, RTL=6. :contentReference[oaicite:0]{index=0}
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
        // Use MAV_CMD_DO_SET_MODE (176): param1 = base_mode, param2 = custom_mode. :contentReference[oaicite:1]{index=1}
        // base_mode: set custom mode enabled
        const uint MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1u;

        uint custom = CopterCustomMode(modeName);
        if (custom == 0) return;

        SendCommandLong(
            stream,
            command: 176, // MAV_CMD_DO_SET_MODE
            p1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2: custom,
            p3: 0, p4: 0, p5: 0, p6: 0, p7: 0
        );
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
        stream.Write(pkt, 0, pkt.Length);
    }

    private static void SendDoSetServo(NetworkStream stream, int servo, int pwm)
    {
        // MAV_CMD_DO_SET_SERVO = 183 :contentReference[oaicite:2]{index=2}
        SendCommandLong(stream, 183, p1: servo, p2: pwm, p3: 0, p4: 0, p5: 0, p6: 0, p7: 0);
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
        stream.Write(pkt, 0, pkt.Length);
    }

    // ----------------- RX drain + heartbeat learn -----------------
    private static void RxDrain(NetworkStream stream)
    {
        var b = new byte[4096];
        while (_rxDrainRun)
        {
            try
            {
                if (!stream.DataAvailable) { Thread.Sleep(10); continue; }
                _ = stream.Read(b, 0, b.Length); // just drain
            }
            catch { Thread.Sleep(50); }
        }
    }

    private static void WaitHeartbeat(NetworkStream stream, int timeoutMs)
    {
        // Minimal MAVLink v1/v2 frame scan just to learn sysid/compid from HEARTBEAT
        // If it fails, we keep default 1/1.
        long end = Environment.TickCount64 + timeoutMs;
        var one = new byte[1];

        // very small state machine for MAVLink1 only (0xFE)
        int state = 0;
        int len = 0, idx = 0;
        byte sys = 1, comp = 1, msgid = 0;
        var payload = new byte[255];

        while (Environment.TickCount64 < end)
        {
            if (!stream.DataAvailable) { Thread.Sleep(5); continue; }
            int r = stream.Read(one, 0, 1);
            if (r != 1) continue;
            byte c = one[0];

            switch (state)
            {
                case 0: // stx
                    if (c == 0xFE) { state = 1; }
                    break;
                case 1: // len
                    len = c;
                    idx = 0;
                    state = 2; // seq next
                    break;
                case 2: // seq
                    state = 3;
                    break;
                case 3: // sysid
                    sys = c;
                    state = 4;
                    break;
                case 4: // compid
                    comp = c;
                    state = 5;
                    break;
                case 5: // msgid
                    msgid = c;
                    if (len == 0) state = 7; else state = 6;
                    break;
                case 6: // payload
                    payload[idx++] = c;
                    if (idx >= len) state = 7;
                    break;
                case 7: // cka
                    state = 8;
                    break;
                case 8: // ckb
                    // if heartbeat msgid=0, accept (we’re not validating CRC here)
                    if (msgid == 0)
                    {
                        _targetSys = sys;
                        _targetComp = comp;
                        return;
                    }
                    state = 0;
                    break;
            }
        }
    }

    // ----------------- utils -----------------
    private static bool Same(ushort[] a, ushort[] b)
    {
        for (int i = 0; i < 8; i++) if (a[i] != b[i]) return false;
        return true;
    }

    private readonly record struct AxisCfg(string Name, int BufIndex, int Min, int Center, int Max, int RcIndex1Based);
}
