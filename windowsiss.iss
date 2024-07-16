; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define MyAppName "WPIcal"
#define MyAppVersion "1.0"
#define MyAppPublisher "WPIcal"
#define MyAppURL "https://github.com/ElliotScher/wpical"
#define MyAppExeName "WPIcal.exe"

[Setup]
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{6D38B811-E4B8-43FC-A00C-6257A2EA9C7A}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf}\{#MyAppName}
; "ArchitecturesAllowed=x64compatible" specifies that Setup cannot run
; on anything but x64 and Windows 11 on Arm.
ArchitecturesAllowed=x64compatible
; "ArchitecturesInstallIn64BitMode=x64compatible" requests that the
; install be done in "64-bit mode" on x64 or Windows 11 on Arm,
; meaning it should use the native 64-bit Program Files directory and
; the 64-bit view of the registry.
ArchitecturesInstallIn64BitMode=x64compatible
DisableProgramGroupPage=yes
; Uncomment the following line to run in non administrative install mode (install for current user only.)
;PrivilegesRequired=lowest
OutputBaseFilename=mysetup
Compression=lzma
SolidCompression=yes
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "build\Release\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\abseil_dll.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\avcodec-60.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\avformat-60.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\avutil-58.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\ceres.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\gflags.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\glog.dll"; DestDir: "{app}"; Flags: ignoreversion
; Source: "build\Release\imgui.ini"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\jpeg62.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\liblzma.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\libpng16.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\libprotobuf.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\libsharpyuv.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\libwebp.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\libwebpdecoder.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_aruco4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_calib3d4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_core4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_dnn4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_features2d4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_flann4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_highgui4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_imgcodecs4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_imgproc4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_objdetect4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\opencv_videoio4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\swresample-4.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\swscale-7.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\tiff.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "build\Release\zlib1.dll"; DestDir: "{app}"; Flags: ignoreversion
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

