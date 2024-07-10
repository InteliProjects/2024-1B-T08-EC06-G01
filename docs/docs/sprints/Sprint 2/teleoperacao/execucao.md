---
title: Instruções de Execução
sidebar_position: 1
---

# Introdução

Este documento descreve como executar o projeto. Ele contém instruções para instalar as dependências necessárias e executar o projeto e serve como guia para configurar tanto o robô quanto o computador que irá controlá-lo.

## Versionamento de Dependências

O projeto foi desenvolvido e testado usando as seguintes versões de dependências:

- **ROS2**: `humble`
- **Python**: `3.10.12`
- **Typer**: `0.9.0`
- **InquirerPy**: `0.3.4`

## Preparo do Ambiente de Execução

:::warning
Para garantir a execução correta do software, é **essencial** preparar adequadamente o ambiente de execução. Isso evitará problemas e erros durante a operação do robô.
:::

### Configuração do Sistema Operacional

O software foi desenvolvido e testado no sistema operacional Ubuntu 20.04 LTS. Recomenda-se o uso deste sistema operacional para garantir a compatibilidade e execução correta do software.

:::tip
Caso deseje utilizar outro sistema operacional, consulte a documentação oficial do ROS2 para verificar a compatibilidade e instruções de instalação. A versão correta do Ubuntu encontra-se [aqui](https://releases.ubuntu.com/jammy/)
:::

### Verificação das Dependências

Antes de executar o projeto, é necessário verificar se todas as dependências estão instaladas corretamente. Para isso, execute os seguintes comandos:

```bash
python3 --version

printenv ROS_DISTRO
```

Os comandos acima devem retornar as versões corretas do Python e do ROS2. Caso contrário, instale as dependências necessárias.

:::warning
Para instalar o ROS2, caso ainda não esteja configurado, siga as instruções detalhadas disponíveis neste [link](https://docs.ros.org/en/humble/Installation.html).
:::

### Estabelecimento de uma Conexão ao Wi-Fi

Para garantir a comunicação entre o robô e o computador, é necessário estabelecer uma conexão Wi-Fi estável. Certifique-se de que o robô e o computador estejam conectados à mesma rede Wi-Fi.

:::tip
Para conectar o robô à rede Wi-Fi, utilize o comando `nmcli` no terminal do robô. Para mais informações, consulte [essa documentação](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/7/html/networking_guide/sec-configuring_ip_networking_with_nmcli).
:::

## Inicialização do Robô

Para iniciar o robô, siga os seguintes passos:

1. Ligue o robô pressionando o botão de liga/desliga.
2. Aguarde até que o robô esteja pronto para ser controlado. Isso pode levar alguns minutos.
3. Conecte-se ao robô via SSH. Para isso, utilize o seguinte comando:

```bash
ssh <USUARIO_DO_ROBO>@<IP_DO_ROBÔ>
```

:::tip
Substitua `<IP_DO_ROBÔ>` pelo endereço IP do robô. Caso não saiba o endereço IP do robô, conecte-o a um monitor e verifique o endereço IP na tela usando o comando `ip addr`.

Substitua `<USUARIO_DO_ROBO>` pelo nome de usuário do robô. Para obter o nome de usuário, conecte o robô a um monitor e obtenha o nome de usuário usando o comando `whoami`.
:::

4. Inicie o ROS2 no robô. Para isso, execute o seguinte comando:

```bash
source /opt/ros/humble/setup.bash
```

5. Inicie o subscriber do robô. Para isso, execute o seguinte comando:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

## Instalação da CLI

Clone o repositório do projeto:

```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G01.git
```

Entre na pasta do projeto:

```bash
cd 2024-1B-T08-EC06-G01/src
```

Rode o script de instalação e execução automático:

```bash
chmod +x ros-run.bash
./ros-run.bash
```

:::warning
Garanta que o robô esteja ligado, inicializado e conectado à mesma rede Wi-Fi que o computador antes de executar o script.
:::

O comando acima irá instalar todas as dependências necessárias para rodar o projeto.

### Explicação do Script de Instalação

```bash
#!/bin/bash

# if the directory "env" does not exist, create a virtual environment
if [ ! -d "env" ]; then
    echo "Creating virtual environment..."
    python3 -m venv env
fi

source env/bin/activate

echo "Installing dependencies..."
pip install -r requirements.txt > /dev/null

cd meu_workspace

# get the site-packages path by getting pip show setuptools (setuptools always comes with pip)
VENV_PATH=$(pip show setuptools | grep "Location: " | awk '{print $2}')

export PYTHONPATH="$PYTHONPATH:$VENV_PATH"

echo "Building package..."
colcon build > /dev/null

source install/setup.bash

ROS_DOMAIN_ID=69 ros2 run cannabot cannabot
```

O script acima faz o seguinte:

1. Verifica se o diretório `env` existe. Se não existir, ele cria um ambiente virtual Python.
2. Ativa o ambiente virtual.
3. Instala as dependências do projeto.
4. Navega até o diretório `meu_workspace`.
5. Obtém o caminho do site-packages executando `pip show setuptools` e exporta o caminho para a variável de ambiente `PYTHONPATH`.
6. Compila o pacote ROS usando o `colcon`.
7. Ativa o pacote ROS.
8. Executa o pacote ROS `cannabot` que é a CLI que controla o robô usando o `ROS_DOMAIN_ID` `69`.
