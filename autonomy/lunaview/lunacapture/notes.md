2023-03-23 Command for Database Json Value Call
===============================================
```
"test_cpp=# SELECT id,robot_json->'power_dump' as power_dump,created_at FROM test_conn";

\c test_cpp
```

2023-05-14 Process to Connect Robot to Laptop
=============================================

## Files found in
```
/etc/postgresql/14/main
```

## `pg_hba.conf` changes

#### Add to last row
```
host    test_cpp        all             10.10.10.101/32         md5
```

## `postgresql.conf` changes
```
listen_addresses = '*'          # what IP address(es) to listen on;
```

2023-05-31 Process to Import CSV File Into Database
===================================================
```
copy test_conn(id, instance_num, robot_json, created_at)
from '/tmp/12.csv'
DELIMITER ','
CSV HEADER;
```

2023-06-08 How to DROP TABLE when a process has locked it
=========================================================
```
SELECT pid
  FROM pg_locks l
  JOIN pg_class t ON l.relation = t.oid AND t.relkind = 'r'
 WHERE t.relname = 'test_conn';
 ```

 Then at normal user terminal, issue `sudo kill <insert pid>` for every pid.
