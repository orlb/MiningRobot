#!/bin/sh
# Create (or update) the postgres database used to store robot data.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo mkdir file_exports

sudo -su postgres << EOF

psql << SQL

\c test_cpp
\copy (SELECT * FROM test_conn WHERE instance_num = $1) TO '/tmp/$1.json' WITH DELIMITER ','

SQL

EOF

sudo mv /tmp/$1.json $SCRIPT_DIR/file_exports/
