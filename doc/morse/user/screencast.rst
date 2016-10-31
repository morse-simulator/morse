Screencast
==========

Many users have asked how to make a screencast of their simulation. Here
we show one solution to this question.

As mentioned in `Ubuntu's wiki <https://wiki.ubuntu.com/ScreenCasts>`_, you can
use may ways to record your desktop, from `VLC <http://www.videolan.org>`_ to
`ffmpeg <https://trac.ffmpeg.org/wiki/Capture/Desktop>`_. Here we will
show how to use
`recordmydesktop <http://recordmydesktop.sourceforge.net/>`_. This is a
command line tool, although a graphical user interface version is also
available (gtk-recordmydesktop).

Install using::

    sudo apt-get install recordmydesktop

Here is an usefull alias to select a window to record::

    alias scast='recordmydesktop --v_bitrate 5000000 --full-shots --fps 10 --no-sound --windowid $(xwininfo | grep "Window id:" | sed -e "s/xwininfo\:\ Window id:\ // ;s/\ .*//")'


The most efficient way to capture is to use low compression,
so then you must know how to re-encode your video before sending it.

The official recommendations from
`YouTube <https://support.google.com/youtube/answer/1722171>`_ and
`Vimeo <https://vimeo.com/help/compression>`_ (as of January 2015)
are: MP4 + H.264 + AAC-LC.

You can use to following command to re-encode properly::

    avconv -i out.ogv -vcodec libx264 -s hd720 -b 5000k -an out.mp4

Or, if you use sound::

    avconv -i out.ogv -vcodec libx264 -acodec libfaac -s hd720 -b 5000k -ab 320k out.mp4
