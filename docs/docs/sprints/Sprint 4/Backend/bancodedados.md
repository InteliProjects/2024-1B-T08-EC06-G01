---
title: "Banco de dados"
sidebar_position: 2
description: Nessa secção iremos mostrar como realizamos a construção do banco de dados do projeto versão 2.0.
---

# Documentação do Banco de Dados

## Introdução

&emsp;Nesta seção, detalharemos a estrutura e a configuração do banco de dados utilizado no projeto. Utilizamos PostgreSQL como nosso sistema de gerenciamento de banco de dados relacional, juntamente com o FastAPI para a construção da API e Ormar como ORM (Object-Relational Mapping) para manipulação de dados.

## Tabelas existentes 

### Tabela `users`:

&emsp;A tabela `users` armazena informações dos usuários do sistema.

- **id**: Identificador único do usuário (chave primária).
- **username**: Nome de usuário.
- **password**: Senha do usuário.
- **admin**: Indica se o usuário é administrador (true/false).

```sql
CREATE TABLE users (
  id serial PRIMARY KEY,
  username text,
  password text,
  admin bool
);
```

### Tabela `robot`:

&emsp;A tabela `robot` armazena informações dos robôs associados aos usuários.

- **id**: Identificador único do robô (chave primária).
- **name**: Nome do robô.
- **user_id**: Identificador do usuário ao qual o robô está associado (chave estrangeira referenciando users.id).

```sql
CREATE TABLE robot (
  id serial PRIMARY KEY,
  name text,
  user_id integer REFERENCES users (id)
);
```

### Tabela `media`:

&emsp;A tabela `media` armazena informações sobre mídias associadas aos robôs.

- **uuid**: Identificador único da mídia (chave primária).
- **title**: Título da mídia.
- **type**: Tipo da mídia (true para vídeo, false para imagem).
- **date**: Data de criação da mídia.
- **robot_id**: Identificador do robô ao qual a mídia está associada (chave estrangeira referenciando robot.id).

```sql
CREATE TABLE media (
  uuid uuid PRIMARY KEY,
  title text,
  type bool,
  date timestamp,
  robot_id integer REFERENCES robot (id)
);
```

### Tabela `log`:

&emsp;A tabela `log` armazena registros de ações realizadas por usuários ao utilizarem a aplicação.

- **id**: Identificador único do log (chave primária).
- **date**: Data em que a ação foi realizada.
- **emergency_button**: Se foi realizado o uso da parada de emergência durante o uso.
- **ia_request**: Para registrar se foi necessário utilizar a visão computacional.
- **username**: Identificar quem realizou as ações durante o uso da aplicação.
- **user_id**: Chave estrangeira para referenciar o usuário que fez uso da aplicação.

```sql 
CREATE TABLE log (
  id serial PRIMARY KEY,
  emergency_button bool,
  ia_request bool,
  username text,
  date timestamp,
  user_id integer REFERENCES users (id)
);
```

### Tabela `temperature`:

&emsp;A tabela `temperature` armazena registro de temperatura medido pelo robô durante o seu uso.

- **id**: Identificador único da temperatura (chave primária).
- **temp**: Valor capturado pelo sensor do robô para medir a temperatura.
- **robot_id**: Chave estrangeira para referenciar que robô foi utilizado.

```sql
CREATE TABLE temperature (
  id serial PRIMARY KEY,
  temp float,
  robot_id integer REFERENCES robot (id)
);
```

### Tabela `location`:

&emsp;A tabela `location` armazena a localização do robô no eixo **x** e **y** durante o seu uso na caldeira.

- **id**: Identificador único da localização (chave primária).
- **location_x**: Valor que representa a localização do robô no eixo **x**.
- **location_y**: Valor que representa a localização do robô no eixo **y**.
- **robot_id**: Chave estrangeira para referenciar que robô foi utilizado.

```sql
CREATE TABLE location (
  id serial PRIMARY KEY,
  location_x float,
  location_y float,
  robot_id integer REFERENCES robot (id)
);
```

## Representação visual do Schema

&emsp;Abaixo está a representação visual do schema do banco de dados utilizando a sintaxe ERD (Entity-Relationship Diagram). Esta representação ajuda a compreender as relações entre as tabelas e como os dados estão organizados.

```sql
erDiagram
    USERS {
        int id
        text username
        text password
        bool admin
    }
    ROBOT {
        int id
        text name
        int user_id
    }
    MEDIA {
        uuid uuid
        text title
        bool type
        timestamp date
        int robot_id
    }
    TEMPERATURE {
        int id
        float temp
        int robot_id
    }
    LOCATION {
        int id
        float location_x
        float location_y
        int robot_id
    }
    LOG {
        int id
        bool emergency_button
        bool ia_request
        text username
        timestamp date
        int user_id
    }

    USERS ||--o{ ROBOT : "possui"
    ROBOT ||--o{ MEDIA : "possui"
    ROBOT ||--o{ TEMPERATURE : "registra"
    ROBOT ||--o{ LOCATION : "registra"
    USERS ||--o{ LOG : "gera"
```

### Explicação das Relações

- **USERS e ROBOT**: Um usuário pode possuir múltiplos robôs, mas cada robô está associado a um único usuário. Esta relação é representada como `USERS ||--o{ ROBOT`.

- **ROBOT e MEDIA**: Cada robô pode ter várias mídias associadas, mas cada mídia está associada a um único robô. Esta relação é representada como `ROBOT ||--o{ MEDIA`.

- **ROBOT e TEMPERATURE**: Cada robô pode registrar múltiplas temperaturas, mas cada registro de temperatura está associado a um único robô. Esta relação é representada como `ROBOT ||--o{ TEMPERATURE`.

- **ROBOT e LOCATION**: Cada robô pode registrar múltiplas localizações, mas cada registro de localização está associado a um único robô. Esta relação é representada como `ROBOT ||--o{ LOCATION`.

- **USERS e LOG**: Um usuário pode gerar múltiplos logs, mas cada log está associado a um único usuário. Esta relação é representada como `USERS ||--o{ LOG.`

&emsp;O diagrama acima mostra como as tabelas estão conectadas umas às outras através de chaves estrangeiras. Ele fornece uma visão clara de como os dados fluem entre as tabelas e como elas se relacionam umas com as outras.