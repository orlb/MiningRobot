2023-03-23 Command for Database Json Value Call
===============================================

"test_cpp=# SELECT id,robot_json->'power_dump' as power_dump,created_at FROM test_conn";

\c test_cpp

2023-05-14 Process to Connect Robot to Laptop
=============================================

## Files found in
/etc/postgresql/14/main

## pg_hba.conf changes

#### Add to last row
host    test_cpp        all             10.10.10.101/32         md5

## postgresql.conf changes
listen_addresses = '*'          # what IP address(es) to listen on;
