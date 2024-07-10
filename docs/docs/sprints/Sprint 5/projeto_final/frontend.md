---
title: "Frontend"
sidebar_position: 3
description: Essa secção detalha o frontend finalizado, indicando as mudanças feitras ao decorrer do projeto.
---

&emsp;Ao decorrer das sprints, o grupo Cannabot foi construindo e aprimorando o frontend proposto para uso completo da solução. Tendo o mockup, proposto na segunda sprint, como base, a aplicação foi desenvolvida utilizando Vue com Vite e Tailwind, ademais, conceitos de UX foram aplicados durante a construção do frontend de modo que a interface fosse feita da melhor forma possível para o usuário final. Sendo assim, as telas são as seguintes:

## Tela de Cadastro

&emsp;O fluxo inicial da aplicação começa com a tela de cadastro, onde não é necessário o uso de email para criar uma conta no sistema:

<div align="center">

![](../../../../..\docs\static\img\sprint5\cadastro.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

## Tela de Login

&emsp;Também, há uma tela de login para usuários já cadastrados:

<div align="center">

![](../../../../..\docs\static\img\sprint5\login.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;O objetivo do grupo criando essas duas telas iniciais é facilitar o controle do que foi feito e, principalmente, de quem fez tal ação. 

## Sidebar

&emsp;Depois de passar pelo fluxo de cadastro e login, o usuário tem acesso ao sistema. Antes de tudo, é necessário mencionar que todas as páginas (com exceção da página de Cadastro e Login) possuem uma sidebar que permite a navegação por todas as funções de uma forma simplificada, ademais, também possui a função de sair da conta que está logada.

<div align="center">

![](../../../../..\docs\static\img\sprint5\homeside.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

## Tela Inicial ou Histórico de Dados

&emsp;Ao entrar na conta, o fluxo continua na tela inicial que apresenta o histórico de dados e o último heatmap gerado:

<div align="center">

![](../../../../..\docs\static\img\sprint5\home.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;Essa tela apresenta, também, uma tabela de dados referente às informações coletadas pelo robô durante alguma operação feita por ele.

## Central de Controle

&emsp;O controle do robô acontece pela tela de ‘Central de Controle’, onde é exibido a visão do robô em tempo real, sendo, também, apresentada a temperatura atual captada pelo sensor de temperatura. Ademais, a interface possui mais dois botões, um referenciado por ‘Modo de Emergência’ - faz com que o robô encerre de imediato sua operação - e outro chamado ‘Modo Autônomo’, onde, - sendo fora do escopo do MVP, portanto, função não implementada (sugestão para os próximos passos) - é ativado a operação independente, deixando a operação teleoperada de lado.

<div align="center">

![](../../../../..\docs\static\img\sprint3\movement.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;Também, durante o modo teleoperado, o controle da máquina deve ser feito através das teclas de seta do teclado, e essa operação é apontada através das teclas adicionadas na interface e, quando pressionadas através do teclado, a cor das setas se altera, indicando que o comando foi respondido.

## Registros de Uso

&emsp;Ainda navegando pela sidebar, chega-se na tela onde exibe os registros de uso da solução (informações que só são possíveis serem apresentadas por conta da página de login):

<div align="center">

![](../../../../..\docs\static\img\sprint5\uso.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

## Histórico de Dados:

&emsp;Finalizando o fluxo (tendo os ícones da sidebar como referência de navegação), voltamos para a tela inicial de histórico de dados, explicada anteriormente.

<div align="center">

![](../../../../..\docs\static\img\sprint5\home.png)
Fonte: Elaborado pelo grupo Cannabot
</div>