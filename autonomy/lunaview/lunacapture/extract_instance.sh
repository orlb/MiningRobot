#!/bin/sh
# Create (or update) the postgres database used to store robot data.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo mkdir $SCRIPT_DIR/file_exports

sudo -su postgres << EOF

psql << SQL

\c test_cpp
\COPY (SELECT * FROM test_conn WHERE instance_num = $1) TO '/tmp/$1.csv' WITH DELIMITER ',' CSV HEADER

SQL

EOF

sudo mv /tmp/$1.csv $SCRIPT_DIR/file_exports/
