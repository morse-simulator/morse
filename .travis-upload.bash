#!/bin/bash
if [ -z "$1" ]; then
  echo "usage: $0 <file>"
  exit 1
fi

name=test$RANDOM@test$RANDOM.fr
pass=$RANDOM
file="$1"
out=$(mktemp)

#tail -f $out &
#tailpid=$!

ftp -pinv > $out << EOF
open dl.free.fr
user $name $pass
bin
put "$file" "$(basename $file)"
close
bye
EOF

#kill $tailpid

url=$(grep -E '226-' $out | grep -Eo 'http://dl.free.fr/[^ ]+')
if [ -n "$url" ]; then
    echo "-- SUCCESS --"
    echo "   url: $url "
else
    echo "-- ERROR --"
    cat $out
fi
rm $out

