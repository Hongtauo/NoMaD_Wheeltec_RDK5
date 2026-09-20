// Narrow OpenSSH askpass adapter. No password is embedded in this executable.
// CREDENTIALW and ownership: https://learn.microsoft.com/en-us/windows/win32/api/wincred/nf-wincred-credreadw
// Blob layout: https://learn.microsoft.com/en-us/windows/win32/api/wincred/ns-wincred-credentialw
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

internal static class RobotSshAskpass
{
    private const string Target = "SocialEmbodiedAgent/robot/wheeltec@192.168.0.100";
    private const string User = "wheeltec";
    private const string Prompt = "wheeltec@192.168.0.100's password:";
    private const uint GenericCredential = 1;
    private const uint FileTypePipe = 3;

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct Credential
    {
        public uint Flags;
        public uint Type;
        public IntPtr TargetName;
        public IntPtr Comment;
        public System.Runtime.InteropServices.ComTypes.FILETIME LastWritten;
        public uint CredentialBlobSize;
        public IntPtr CredentialBlob;
        public uint Persist;
        public uint AttributeCount;
        public IntPtr Attributes;
        public IntPtr TargetAlias;
        public IntPtr UserName;
    }

    [DllImport("advapi32.dll", EntryPoint = "CredReadW", CharSet = CharSet.Unicode,
        ExactSpelling = true, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool CredRead(string target, uint type, uint flags,
        out IntPtr credential);

    [DllImport("advapi32.dll", ExactSpelling = true)]
    private static extern void CredFree(IntPtr credential);

    [DllImport("kernel32.dll", ExactSpelling = true)]
    private static extern IntPtr GetStdHandle(int standardHandle);

    [DllImport("kernel32.dll", ExactSpelling = true)]
    private static extern uint GetFileType(IntPtr handle);

    private static bool AllowedInvocation(string[] args)
    {
        return args.Length == 1 && args[0] != null && args[0].Trim() == Prompt
            && Environment.GetEnvironmentVariable("SSH_ASKPASS_REQUIRE") == "force"
            && Environment.GetEnvironmentVariable("NOMAD_ROBOT_ASKPASS") == "1"
            && String.IsNullOrEmpty(Environment.GetEnvironmentVariable("SSH_ASKPASS_PROMPT"))
            && Console.IsOutputRedirected
            && GetFileType(GetStdHandle(-11)) == FileTypePipe;
    }

    private static bool ValidPassword(char[] password)
    {
        if (password.Length == 0 || password.Length > 512)
            return false;
        foreach (char value in password)
        {
            if (value == '\0' || value == '\r' || value == '\n'
                || value == '\u0085' || value == '\u2028' || value == '\u2029')
                return false;
        }
        return true;
    }

    private static int Main(string[] args)
    {
        IntPtr allocation = IntPtr.Zero;
        Credential credential = new Credential();
        byte[] blob = null;
        char[] password = null;
        byte[] encoded = null;
        try
        {
            // Never answer unknown-host confirmation, key passphrases, or other users.
            if (!AllowedInvocation(args))
                return 1;
            if (!CredRead(Target, GenericCredential, 0, out allocation))
                return 1;
            credential = (Credential)Marshal.PtrToStructure(allocation, typeof(Credential));
            if (credential.Type != GenericCredential
                || !String.Equals(Marshal.PtrToStringUni(credential.TargetName), Target,
                    StringComparison.OrdinalIgnoreCase)
                || !String.Equals(Marshal.PtrToStringUni(credential.UserName), User,
                    StringComparison.Ordinal)
                || credential.CredentialBlob == IntPtr.Zero
                || credential.CredentialBlobSize == 0
                || credential.CredentialBlobSize > 1024
                || credential.CredentialBlobSize % 2 != 0)
                return 1;
            blob = new byte[(int)credential.CredentialBlobSize];
            Marshal.Copy(credential.CredentialBlob, blob, 0, blob.Length);
            password = new UnicodeEncoding(false, false, true).GetChars(blob);
            if (!ValidPassword(password))
                return 1;
            encoded = new UTF8Encoding(false, true).GetBytes(password);
            // Write bytes directly: no BOM, shell interpolation, logs, or terminal echo.
            using (Stream output = Console.OpenStandardOutput())
            {
                output.Write(encoded, 0, encoded.Length);
                output.WriteByte(10);
                output.Flush();
            }
            return 0;
        }
        catch
        {
            // Exceptions must not expose a credential or unexpected prompt.
            return 1;
        }
        finally
        {
            if (encoded != null) Array.Clear(encoded, 0, encoded.Length);
            if (password != null) Array.Clear(password, 0, password.Length);
            if (blob != null) Array.Clear(blob, 0, blob.Length);
            if (allocation != IntPtr.Zero)
            {
                try
                {
                    // The CredRead allocation is our private copy, not stored credentials.
                    if (credential.CredentialBlob != IntPtr.Zero
                        && credential.CredentialBlobSize <= 2560)
                    {
                        for (int index = 0; index < (int)credential.CredentialBlobSize; index++)
                            Marshal.WriteByte(credential.CredentialBlob, index, 0);
                    }
                }
                catch { }
                finally { CredFree(allocation); }
            }
        }
    }
}
