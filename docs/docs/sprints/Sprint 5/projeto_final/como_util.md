---
title: Como utilizar a solução
sidebar_position: 2
---

# Introdução

Nesta página é possível entender como instalar e rodar o projeto no seu computador pessoal. 

## Versionamento de Dependências

O projeto foi desenvolvido e testado usando as seguintes versões de dependências:

- **ROS2**: `humble`
- **Python**: `3.10.12`
- **Typer**: `0.9.0`
- **InquirerPy**: `0.3.4`
- **FastAPI**: `0.68.0`
- **VUE**: `3.2.0`
- **YOLO**: `8.0`
  

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

Nessa sprint, arrumamos a execução da aplicação web em um docker e estamos utilizando o docker-compose para subir o banco de dados, backend e frontend. Então, agora temos 2 passos para executar o projeto, sendo eles: subir a aplicação web e executar os códigos do robô.

## Aplicação Web

Para executar a aplicação web, siga os seguintes passos:

1. Clone o repositório do projeto:
```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G01.git

cd 2024-1B-T08-EC06-G01/src
```
2. Execute o docker-compose para subir o tudo:
```bash
docker-compose up
```

Caso não tenha o docker configurado, siga os passos abaixo:

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

Caso esses códigos não estejam sendo executados, execute os seguintes comandos:

Em um terminal do robô:
```bash
cd 2024-1B-T08-EC06-G01/src/package
bash run_camera.sh
```

Em outro terminal do robô:
```bash
cd 2024-1B-T08-EC06-G01/src/package
bash run_websocket.sh
```

# Utilização

Após seguir os passos acima, você pode acessar o frontend e controlar o robô. Acesse o frontend em `http://localhost:80` para teleoperar o robô e ver as imagens da câmera.