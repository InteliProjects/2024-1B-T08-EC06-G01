---
title: Como utilizar a solução
sidebar_position: 1
---

# Introdução

Este documento descreve como utilizar a solução. Ele contém instruções para instalar as dependências necessárias e executar o projeto e serve como guia para configurar tanto o robô quanto o computador que irá controlá-lo.


## Versionamento de Dependências

O projeto foi desenvolvido e testado usando as seguintes versões de dependências:

- **ROS2**: `humble`
- **Python**: `3.10.12`
- **Typer**: `0.9.0`
- **InquirerPy**: `0.3.4`
- **FastAPI**: `0.68.0`
- **VUE**: `3.2.0`
  

# Preparo do Ambiente de Execução

:::warning
Para garantir a execução correta do software, é **essencial** preparar adequadamente o ambiente de execução. Isso evitará problemas e erros durante a operação do robô.
:::

# Configuração do Sistema Operacional

Tudo envolvido com o ROS2 foi desenvolvido e testado no sistema operacional Ubuntu 20.04 LTS. Recomenda-se o uso deste sistema operacional para garantir a compatibilidade e execução correta das partes que possuem ROS2.

:::tip
Caso deseje utilizar outro sistema operacional, consulte a documentação oficial do ROS2 para verificar a compatibilidade e instruções de instalação. A versão correta do Ubuntu encontra-se [aqui](https://releases.ubuntu.com/jammy/)
:::

# Execução do Projeto

Temos 3 partes principais para a execução do projeto, sendo elas: o robô, o backend e o fronted.

## Robô

Para iniciar o robô, siga os seguintes passos:

1. Ligue o robô atráves do switch.
2. Aguarde até que o robô esteja pronto para ser controlado. Isso pode levar alguns minutos.
3. (Opcional) Conecte-se ao robô via SSH, caso deseje ver log de algo. Para isso, utilize o seguinte comando:

```bash
ssh <USUARIO_DO_ROBO>@<IP_DO_ROBÔ>
```

:::tip
Substitua `<IP_DO_ROBÔ>` pelo endereço IP do robô. Caso não saiba o endereço IP do robô, conecte-o a um monitor e verifique o endereço IP na tela usando o comando `ip addr`.

Substitua `<USUARIO_DO_ROBO>` pelo nome de usuário do robô. Para obter o nome de usuário, conecte o robô a um monitor e obtenha o nome de usuário usando o comando `whoami`.
:::

Quando o robô for ligado, ele automaticamente inicia o ROS2 e um webscoket server para comunicação com o backend, para receber comandos e enviar informações de sensores e outro websocket para enviar os dados da câmera para o frontend. Os códigos que são executados no robô estão localizados em `/src/package/camera` e `/src/package/workspace/src/websocket_robot`.

## Backend

Para iniciar o backend, siga os seguintes passos:

1. Instale o Docker Engine:
```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
```
2. Clone o repositório do backend e vá para o diretório src do projeto e rode o banco de dados:
```bash
docker compose up postgres
```
3. Abra outro terminal e va para a pasta do backend (repo/src/backend)
4. Crie uma virtualenv e instale as dependências requirements.txt:
```bash
python3 -m env env
source venv/bin/activate
pip install -r requirements.txt
```
5. Crie um arquivo .env e adicione as variáveis de ambiente:
```bash
#! POSTGRES
# Formato: postgresql://<username>:<password>@<host>:<port>/<database>
DATABASE_URL="postgresql://cannabot:Cnh39pKWBV4mNpzcxMcBdLxud5nJxKzWsq6iXstdeLf2vZbCHKtAn36w8wqBmFHL@localhost:5432/cannabot"

#! BUCKET
BUCKET_HOST="temp"
BUCKET_PORT="9000"
BUCKET_ACCESS_KEY="temp"
BUCKET_SECRET_KEY="temp"
BUCKET_USE_SSL="false"

#! SECRETS

#* Usado para criptografar e descriptografar tokens JWT
# Obtido usando: openssl rand -hex 32
JWT_SECRET = "f6013799590e9d59c6853253a2a7af2dc88a3b36f891ede916a82e02a87a8555"

#* Usado para criptografar e descriptografar informações sensíveis no banco de dados usando AES256
# Obtido usando: openssl rand -base64 32
AES_SECRET = "wAhXwSFyvh7i8hE0/DdqeJS490WFyAOImaE/yZACHzI="
```
6. Rode o backend:
```bash
python3 src/main.py
```

## Frontend

Para iniciar o frontend, siga os seguintes passos:

1. Vá para a pasta do frontend (repo/src/frontend)
2. Instale as dependências:
```bash
npm install
```
3. Rode o frontend:
```bash
npm run serve
```
4. Acesse o frontend em `http://localhost:8080`

# Utilização

Após seguir os passos acima, você pode acessar o frontend e controlar o robô. Acesse o frontend em `http://localhost:8080/control` para teleoperar o robô e ver as imagens da câmera.