#!/bin/bash

# SSH 时间同步脚本                                           //script de sincronizacao de horario SSH
# 使用方法: ./sync_time.sh [remote_host] [remote_user]      //uso: ./sync_time.sh [host_remoto] [usuario_remoto]

# 配置参数                                                  //configuracao de parametros    
REMOTE_HOST="${1:-192.168.10.101}"  # 远程主机IP，默认值     //IP do host remoto, valor padrao
REMOTE_USER="${2:-master}"        # 远程用户名，默认值       //nome do usuario remoto, valor padrao

# 颜色输出                                                 //codigo para cores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}开始时间同步...${NC}"                     #-e "${YELLOW}inicio da sincronizacao de horario...${NC}"
echo "远程主机: $REMOTE_HOST"                              #"host remoto: $REMOTE_HOST"                
echo "远程用户: $REMOTE_USER"                              #"usuario remoto: $REMOTE_USER"
echo

# 显示当前本机时间                                          //mostra o horario atual da maquina local
echo -e "${YELLOW}当前本机时间:${NC}"
date
echo

# 从远程机器获取时间
echo -e "${YELLOW}获取远程机器时间...${NC}"
REMOTE_TIME=$(ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" 'date +"%Y-%m-%d %H:%M:%S"' 2>/dev/null)

# 检查 SSH 连接是否成功
if [ $? -ne 0 ] || [ -z "$REMOTE_TIME" ]; then
    echo -e "${RED}错误: 无法连接到远程主机 $REMOTE_HOST${NC}"
    echo "请检查:"
    echo "1. 网络连接"
    echo "2. SSH 服务是否运行"
    echo "3. 用户名和主机地址是否正确"
    echo "4. SSH 密钥或密码认证"
    exit 1
fi

echo -e "${GREEN}远程机器时间: $REMOTE_TIME${NC}"
echo

# 更新本机时间
echo -e "${YELLOW}更新本机时间...${NC}"

# 检查是否有 sudo 权限
if ! sudo -n true 2>/dev/null; then
    echo "需要 sudo 权限来更新系统时间，请输入密码:"
fi

# 使用 date 命令设置时间
if sudo date -s "$REMOTE_TIME" >/dev/null 2>&1; then
    echo -e "${GREEN}时间同步成功! 更新后的本机时间: ${NC}"
    date
    echo
else
    echo -e "${RED}时间同步失败!${NC}"
    echo "可能的原因:"
    echo "1. 没有足够的权限"
    echo "2. 时间格式不正确"
    exit 1
fi

echo -e "${GREEN}时间同步完成!${NC}"