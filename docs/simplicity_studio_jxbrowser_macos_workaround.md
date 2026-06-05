# Simplicity Studio 5 — JxBrowser blank Configurator workaround (macOS, Apple Silicon)

Purpose: capture the symptom, root cause, and patch procedure for a Simplicity Studio installation issue that produces blank `.slcp` and `.pintool` editors on Apple Silicon Macs. The patch lives inside the Studio app bundle, so a Studio update will overwrite it and you will need to reapply.

## 1. Symptom

Opening `Creed_DBS_app.slcp` or `Creed_DBS_app.pintool` from the Project Explorer in Simplicity Studio shows a blank editor pane. Other Studio functions (build, debug, file editing) work normally. The issue persists across:

- Project clean and rebuild.
- Deleting `.projectlinkstore` (Silicon Labs known-issue 1320466 fix).
- Deleting `.uceditor/` cache.
- Switching to a fresh workspace and re-importing the project.

## 2. Root cause

Studio uses **JxBrowser** (an embedded Chromium component) to render the HTML-based Configurator editors. The relevant log entries in `<workspace>/.metadata/.log` are:

```
!MESSAGE Incomplete JxBrowser initialization
java.util.concurrent.ExecutionException:
  com.teamdev.jxbrowser.engine.ChromiumBinariesDeliveryException:
  Failed to extract Chromium binaries into /var/folders/.../T/JxBrowser/8.9.0
Caused by: java.io.FileNotFoundException: chromium-mac-arm.7z
```

JxBrowser 8.9.0 (used by Studio 5 as of mid-2025) detects Apple Silicon hardware at runtime and looks for `chromium-mac-arm.7z` inside its plugin jar. The Simplicity Studio installation in question only ships the **x86_64** JxBrowser plugin:

```
plugins/com.silabs.external.jxbrowser.macosx_x86_64_8.9.0.202508262056-53/
  lib/jxbrowser-mac-8.9.0.jar
    └── 8.9.0/chromium-mac.7z          <-- present, no arch suffix
                                         <-- chromium-mac-arm.7z is what JxBrowser asks for
```

Studio itself is also `x86_64` and runs under Rosetta. Older JxBrowser 7.41.2 worked because its loader did not request an arch-suffixed file name. The fix below adds the missing arch-suffixed entry to the jar so JxBrowser finds and extracts a Chromium that Studio (under Rosetta) can run.

## 3. Patch procedure

These commands modify `/Applications/Simplicity Studio.app/...` and require write access to the app bundle (no `sudo` needed if you own the install).

```bash
PLUGIN_LIB="/Applications/Simplicity Studio.app/Contents/Eclipse/plugins/com.silabs.external.jxbrowser.macosx_x86_64_8.9.0.202508262056-53/lib"
JAR="$PLUGIN_LIB/jxbrowser-mac-8.9.0.jar"
WORK="$TMPDIR/jxbrowser_fix_$$"

# 1. Quit Simplicity Studio fully before running.

# 2. Backup the jar.
cp "$JAR" "$JAR.bak"

# 3. Extract the existing chromium-mac.7z and stage it under the arch-suffixed name.
mkdir -p "$WORK/8.9.0"
unzip -p "$JAR" "8.9.0/chromium-mac.7z" > "$WORK/8.9.0/chromium-mac-arm.7z"

# 4. Add the arch-suffixed entry to the jar without recompressing.
( cd "$WORK" && zip -Z store -u "$JAR" "8.9.0/chromium-mac-arm.7z" )
rm -rf "$WORK"

# 5. Clear stale extraction state from a previous failed launch.
rm -rf /var/folders/*/T/JxBrowser/8.9.0

# 6. Launch Studio. First open of .slcp will be slower (~30–60 s) while
#    JxBrowser extracts the ~100 MB Chromium for the first time.
```

Verification: after the first successful open, `/var/folders/.../T/JxBrowser/8.9.0/Chromium.app/...` will exist (mirroring the `7.41.2/` extraction tree).

## 4. Rollback

```bash
PLUGIN_LIB="/Applications/Simplicity Studio.app/Contents/Eclipse/plugins/com.silabs.external.jxbrowser.macosx_x86_64_8.9.0.202508262056-53/lib"
mv "$PLUGIN_LIB/jxbrowser-mac-8.9.0.jar.bak" "$PLUGIN_LIB/jxbrowser-mac-8.9.0.jar"
rm -rf /var/folders/*/T/JxBrowser/8.9.0
```

## 5. When you will need to reapply

- After any Simplicity Studio update that touches the JxBrowser feature (the plugin path will change to a new build number; rerun Section 3 with the new path).
- After a reinstall of Simplicity Studio.
- After a Mac OS reboot **only** if `/var/folders/.../T/JxBrowser/8.9.0/` was wiped (rare; macOS does not aggressively clean `$TMPDIR`).

If the next Simplicity Studio release ships an `aarch64` (Apple Silicon native) JxBrowser plugin alongside the existing x86_64 one — i.e., a plugin directory named `com.silabs.external.jxbrowser.macosx_aarch64_*` appears under `Contents/Eclipse/plugins/` — this workaround should no longer be necessary.

## 6. Permanent alternative

Reinstall Simplicity Studio 5 from a Mac (Apple Silicon) installer that bundles an `aarch64` JxBrowser plugin. This is a clean fix but costs ~30 minutes of reinstalling SDKs (GSDK 4.4.1, 8051 SDK, Simplicity Demos, etc.) and re-importing projects. The jar patch above avoids that disruption while still letting the Configurator render correctly.
