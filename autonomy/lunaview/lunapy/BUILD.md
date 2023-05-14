Build Process
=============

These instructions assums that you already have performed the build process for `lunacapture`, which includes installation of certain required libraries, such as `libpq-dev`.

## Install Python 3.x
```shell
sudo apt-get update
sudo apt-get install python3.7
```

Test that you have Python 3 installed.

```shell
python3 --version
```

## Install Anaconda

[Follow this link to install the Anaconda environment on your system.](https://docs.anaconda.com/free/anaconda/install/linux/)

## Install Python Pip

```shell
sudo apt-get install python3-pip
```

## Install Psycopg2

```shell
sudo pip3 install Psycopg2
```

## Install SQLAlchemy

```shell
sudo pip3 install SQLAlchemy
```
