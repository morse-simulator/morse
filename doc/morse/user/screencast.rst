Screencast
==========

As many questions were raised, we gather here some tips on how-to make beautifull
screencast of your simulation.

As mentioned in `Ubuntu's wiki <https://wiki.ubuntu.com/ScreenCasts>`_, you can
use may way to recorde your desktop, from `VLC <http://www.videolan.org>`_ to
`ffmpeg <https://trac.ffmpeg.org/wiki/Capture/Desktop>`_, we will present
`recordmydesktop <http://recordmydesktop.sourceforge.net/>`_, a command line tool
for which exists as well an interface (gtk-recordmydesktop).

Install using::

    sudo apt-get install recordmydesktop

Here is an usefull alias to select a window to record::

    alias scast='recordmydesktop --v_bitrate 5000000 --full-shots --fps 10 --no-sound --windowid $(xwininfo | grep "Window id:" | sed -e "s/xwininfo\:\ Window id:\ // ;s/\ .*//")'


The most efficient way to capture is to use low compression,
so then you must know how to re-encode your video before sending it.

Official recommendation from
`YouTube <https://support.google.com/youtube/answer/1722171>`_ and
`Vimeo <https://vimeo.com/help/compression>`_ (as of January 2015)
are: MP4 + H.264 + AAC-LC.

You can use to following command to re-encode properly::

    avconv -i out.ogv -vcodec libx264 -s hd720 -b 5000k -an out.mp4

Or, if you use sound::

    avconv -i out.ogv -vcodec libx264 -acodec libfaac -s hd720 -b 5000k -ab 320k out.mp4
