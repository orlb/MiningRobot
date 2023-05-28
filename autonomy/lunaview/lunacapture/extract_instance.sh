#!/bin/sh
# Create (or update) the postgres database used to store robot data.

 if [ -z "$*" ] ; then
     echo "Please provide the instance number that you wish to export as a commandline argument\nFor example: ./extract_instance.sh 1"
     exit 0
 fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo mkdir $SCRIPT_DIR/file_exports

sudo -su postgres << EOF

psql << SQL

\c test_cpp
\COPY (SELECT * FROM test_conn WHERE instance_num = $1) TO '/tmp/$1.csv' WITH DELIMITER ',' CSV HEADER

SQL

EOF

sudo mv /tmp/$1.csv $SCRIPT_DIR/file_exports/

if test -f $SCRIPT_DIR/file_exports/$1.csv; then
    echo "Exported instance number $1 to ./file_exports/$1.csv"
fi
