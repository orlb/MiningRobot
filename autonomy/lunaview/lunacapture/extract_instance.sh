#!/bin/sh
# Extract data from the database according to a provided instance number.

 if [ -z "$*" ] ; then
     echo "Please provide the instance number that you wish to export as a commandline argument\nFor example: ./extract_instance.sh 1"
     exit 0
 fi

INSTANCE="$1"
SCRIPT_DIR=`dirname "$0"`

DESTDIR="$SCRIPT_DIR/file_exports"
mkdir -p "$DESTDIR"
chmod 777 "$DESTDIR"
DEST="$DESTDIR/$INSTANCE.csv"

sudo -su postgres << EOF

psql << SQL

\c test_cpp
\COPY (SELECT * FROM test_conn WHERE instance_num = $INSTANCE) TO '$DEST' WITH DELIMITER ',' CSV HEADER

SQL

EOF

if test -f "$DEST"; then
    echo "Exported instance number $INSTANCE to $DEST"
