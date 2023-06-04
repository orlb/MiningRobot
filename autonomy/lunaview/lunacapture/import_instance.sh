#!/bin/sh
# Import data from file into the database

 if [ -z "$*" ] ; then
     echo "Please provide the instance number that you wish to import as a commandline argument\nFor example: ./import_instance.sh 1"
     exit 0
 fi

INSTANCE="$1"
SCRIPT_DIR=`dirname "$0"`

IMPORTDIR="$SCRIPT_DIR/file_exports"
FILE="$IMPORTDIR/$INSTANCE.csv"

 if [ -z "$FILE" ] ; then
     echo "The file $FILE does not exist in the file_exports directory."
     exit 0
 fi

DESTFILE="/tmp/$INSTANCE.csv" 

cp $FILE $DESTFILE

sudo -su postgres << EOF

psql << SQL

\c test_cpp
DELETE FROM test_conn;
COPY test_conn(id, instance_num, robot_json, created_at) FROM '$DESTFILE' DELIMITER ',' CSV HEADER;

SQL

EOF
