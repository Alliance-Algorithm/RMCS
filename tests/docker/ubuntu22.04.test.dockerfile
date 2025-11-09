# 使用 Ubuntu 22.04 作为基础镜像
FROM ubuntu:22.04

# 设置工作目录
WORKDIR /app

RUN apt-get update && apt-get install -y sudo

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai

RUN apt-get update && \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
    
# 拷贝项目文件
COPY ./env /env

RUN chmod +x /env/ubuntu22.04.sh && /env/ubuntu22.04.sh

COPY . /app


