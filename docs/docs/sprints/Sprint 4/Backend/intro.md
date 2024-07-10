---
title: "Introdução"
sidebar_position: 1
description: Principais atualizações realizadas entre a sprint 3 e a sprint 4 em nosso backend e banco de dados.
---

# Introdução:

&emsp;O objetivo desta seção é contextualizar as principais mudanças na documentação do backend e do banco de dados do nosso projeto entre a sprint 3 e a sprint 4.

## Banco de dados

&emsp;As principais mudanças no banco de dados do projeto foram:

&emsp;1 - Reestruturação da tabela `logs`:
- Decidimos que os logs fariam mais sentido se referenciando às ações dos usuários, em vez das mídias.

&emsp;2 - Criação das tabelas `location` e `temperature`:
- Tabela `location`: Armazena a localização do robô no eixo X e Y durante seu uso.
- Tabela `temperature`: Armazena os dados de temperatura do ambiente em que o robô está operando.

## Backend

&emsp;As principais mudanças no backend do projeto foram:

&emsp;1 - Reestruturação das rotas de `logs`:
- A rota de **POST** de logs foi movida para o arquivo `routes/users.py`.
- Quando um usuário faz login, uma função é chamada para criar um log de novo acesso, que é atualizado conforme as ações do usuário durante seu uso na aplicação.

&emsp;2 - Criação de CRUD's para as novas tabelas:
- Foram criados CRUD's para as tabelas `location` e `temperature`, permitindo a manipulação dos dados inseridos nessas tabelas.


