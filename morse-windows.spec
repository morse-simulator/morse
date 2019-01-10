# -*- mode: python -*-
# spec file for pyinstaller to build Morse for Windows

MorseAny = Analysis(['morserun.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in morserun.py
             datas= [ ('..\\share\\', 'share' ), ('..\\Lib\\site-packages\\morse', 'morse' ), ('..\\Lib\\site-packages\\pymorse', 'pymorse' ) ],
             hookspath=None,
             runtime_hooks=None,
             excludes= [])

Morse_pyz = PYZ(MorseAny.pure)
Morse_exe = EXE(Morse_pyz,
          MorseAny.scripts,
          exclude_binaries=True,
          name='morse.exe',
          debug=False,
          strip=None,
          upx=True,
          console=True )
Morse_coll = COLLECT(Morse_exe,
               MorseAny.binaries,
               MorseAny.zipfiles,
               MorseAny.datas,
               strip=None,
               upx=True,
               name='morserun')
