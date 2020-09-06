using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace ClientAsset {
  public class Subby {
    [DllImport("libclient.so", CharSet=CharSet.Ansi)]
    static extern void subscribe([MarshalAs(UnmanagedType.LPStr)]string topic);
    [DllImport("libclient.so", CharSet=CharSet.Ansi)]
    [return: MarshalAs(UnmanagedType.LPStr)]
    static extern IntPtr sub_getstring();
    [DllImport("libclient.so")]
    static extern void unsubscribe();

    public void Subscribe(string topic) {
      subscribe(topic);
    }
    public string GetString() {
      return Marshal.PtrToStringAnsi(sub_getstring());
    }
    public void Unsubscribe() {
      unsubscribe();
    }

    public static string GetKey(string jsonstr, string key) {
      string keyStr = "\"" + key + "\": ";
      string substr = jsonstr.Substring(jsonstr.IndexOf(keyStr) +
          keyStr.Length);
      int end = substr.IndexOf(",");
      if (end == -1) {
        end = substr.IndexOf("}");
      }
      substr = substr.Substring(0, end);
      return substr;
    }
  }
}
