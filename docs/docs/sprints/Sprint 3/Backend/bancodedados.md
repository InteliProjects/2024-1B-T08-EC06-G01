---
title: "Banco de dados"
sidebar_position: 1
description: Nessa secção iremos mostrar como realizamos a construção do banco de dados do projeto.
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

&emsp;A tabela `log` armazena registros de ações realizadas sobre as mídias.

- **id**: Identificador único do log (chave primária).
- **media_uuid**: Identificador da mídia associada (chave estrangeira referenciando media.uuid).
- **action**: Descrição da ação realizada.
- **date**: Data em que a ação foi realizada.
- **type**: Tipo de ação (true para sucesso, false para falha).

```sql 
CREATE TABLE log (
  id serial PRIMARY KEY,
  media_uuid uuid REFERENCES media (uuid),
  action text,
  date timestamp,
  type bool
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
    LOG {
        int id
        uuid media_uuid
        text action
        timestamp date
        bool type
    }

    USERS ||--o{ ROBOT : "possui"
    ROBOT ||--o{ MEDIA : "possui"
    MEDIA ||--o{ LOG : "possui"
```

### Explicação das Relações

- **USERS e ROBOT**: Um usuário pode possuir múltiplos robôs, mas cada robô está associado a um único usuário. Esta relação é representada como `USERS ||--o{ ROBOT`.

- **ROBOT e MEDIA**: Cada robô pode ter várias mídias associadas, mas cada mídia está associada a um único robô. Esta relação é representada como `ROBOT ||--o{ MEDIA`.

- **MEDIA e LOG**: Cada mídia pode ter múltiplos logs, mas cada log está associado a uma única mídia. Esta relação é representada como `MEDIA ||--o{ LOG`.

&emsp;O diagrama acima mostra como as tabelas estão conectadas umas às outras através de chaves estrangeiras. Ele fornece uma visão clara de como os dados fluem entre as tabelas e como elas se relacionam umas com as outras.