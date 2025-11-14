# MINIO CLI

## Install
```bash
curl -fsSLo /tmp/mc https://dl.min.io/client/mc/release/linux-amd64/mc \
	&& sudo install -m 0755 /tmp/mc /usr/local/bin/mc
```

## Alias to server
```bash
mc alias set [ALIAS] http://[IP]:[PORT] [ACCESS KEY] [SECRET KEY]
```
### Example
```bash
mc alias set local http://127.0.0.1:9000 minioadmin minioadmin
```

## Useful commands
### List buckets
```bash
mc ls local
```

### List objects in bucket
```bash
mc ls --recursive local/public
```
### Upload files
```bash
mc cp ./archivo.txt local/public/
```