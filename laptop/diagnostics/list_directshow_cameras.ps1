param([int]$ProbeIndex = -1)
$cameraSource = @"
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;
[ComImport, Guid("29840822-5B84-11D0-BD3B-00A0C911CE86"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface ICreateDevEnum { [PreserveSig] int CreateClassEnumerator([In] ref Guid category, out IEnumMoniker enumerator, int flags); }
[ComImport, Guid("55272A00-42CB-11CE-8135-00AA004BB851"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IPropertyBag { [PreserveSig] int Read([MarshalAs(UnmanagedType.LPWStr)] string name, [MarshalAs(UnmanagedType.Struct)] out object value, IntPtr errorLog); [PreserveSig] int Write([MarshalAs(UnmanagedType.LPWStr)] string name, [In,MarshalAs(UnmanagedType.Struct)] ref object value); }
public class CameraDevice { public int Index; public string Name; public string DevicePath; public string BindResult; }
public static class CameraEnumerator {
 public static List<CameraDevice> Enumerate(int probeIndex) {
  var result=new List<CameraDevice>();
  object sys=Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("62BE5D10-60EB-11D0-BD3B-00A0C911CE86")));
  IEnumMoniker en=null;
  try {
   Guid cat=new Guid("860BB310-5D01-11D0-BD3B-00A0C911CE86");
   if(((ICreateDevEnum)sys).CreateClassEnumerator(ref cat,out en,0)!=0) return result;
   IMoniker[] mon=new IMoniker[1]; int index=0;
   while(en.Next(1,mon,IntPtr.Zero)==0){
    object obj=null;
    try {
     Guid bagId=new Guid("55272A00-42CB-11CE-8135-00AA004BB851");
     mon[0].BindToStorage(null,null,ref bagId,out obj);
     object name,path; var bag=(IPropertyBag)obj;
     bag.Read("FriendlyName",out name,IntPtr.Zero);
     bag.Read("DevicePath",out path,IntPtr.Zero);
     var item=new CameraDevice { Index=index,Name=Convert.ToString(name),DevicePath=Convert.ToString(path) };
     if(index==probeIndex) {
      object filter=null;
      try { Guid iid=new Guid("56A86895-0AD4-11CE-B03A-0020AF0BA770"); mon[0].BindToObject(null,null,ref iid,out filter); item.BindResult="FILTER_BIND_OK_NOT_STREAM_TESTED"; }
      catch(Exception error) { item.BindResult="0x"+error.HResult.ToString("X8")+" "+error.Message; }
      finally { if(filter!=null) Marshal.ReleaseComObject(filter); }
     }
     result.Add(item);
    } finally { if(obj!=null) Marshal.ReleaseComObject(obj); Marshal.ReleaseComObject(mon[0]); }
    index++;
   }
  } finally { if(en!=null) Marshal.ReleaseComObject(en); Marshal.ReleaseComObject(sys); }
  return result;
 }
}
"@
Add-Type -TypeDefinition $cameraSource
[CameraEnumerator]::Enumerate($ProbeIndex) | ConvertTo-Json -Depth 4
