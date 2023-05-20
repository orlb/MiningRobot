#!/bin/sh
# Drop the postgres database table used to store robot data.

sudo -su postgres << EOF

psql << SQL

\c test_cpp
DROP TABLE test_conn;

SQL

EOF


