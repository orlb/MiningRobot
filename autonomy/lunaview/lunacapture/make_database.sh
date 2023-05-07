#!/bin/sh
# Create (or update) the postgres database used to store robot data.

pass=`cat .dbpass`

sudo -su postgres << EOF

psql << SQL

ALTER USER postgres PASSWORD '$pass';
CREATE DATABASE test_cpp;

SQL

EOF


