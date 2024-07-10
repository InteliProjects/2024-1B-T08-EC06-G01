---
title: "Protótipo da Interface de Usuário"
sidebar_position: 1
---

&emsp;Durante a terceira sprint, o grupo Cannabot deu início ao desenvolvimento das telas do frontend. Todas as telas foram desenvolvidas com base no wireframe e mockup que foram desenvolvidos na última sprint. Durante esta sprint o grupo foi capaz de desenvolver as telas da página inicial com o heatmap, controle do robô e histórico de uso.

Esse desenvolvimento pode ser acompanhado a seguir:

## Telas e Tecnologias 

&emsp; Antes de mais nada é primordial comentar sobre as tecnologias que foram utilizadas para desenvolver o frontend. O grupo opteu por desenvolver as telas utilizando as seguintes tecnologias: cd. 

&emsp; O motivo da escolhas dessas técnologias é que os membros do grupo que ficaram responsáveis por fazer o frontend gostariam de aprender novas tecnologias. Assim, o Vue se torna uma escolha óbvia visto que é um framework muito utilizado na industria, totalmente open source e todo o código é escrito em uma única página. 


#### Tela inicial

&emsp;A tela inicial é onde o usuário pode conferir o histórico das medições de temperatura do robô e o mapa de calor gerado pelos dados do sensor.

&emsp;Assim, é possível entender como anda a temperatura do reboiler e acompanhar as medições ao longo do tempo.

<div align="center"> 

![](../../../../..\docs\static\img\sprint3\home.png)
Fonte: Elaborado pelo grupo Cannabot
</div>


#### Controle do robô

<div align="center"> 

![Rodrigão abaporu](../../../../..\docs\static\img\sprint3\movement.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;Na página de "Central de Controle", o usuário é capaz de conduzir o robô com as teclas W,A,S,D ou as setinhas do teclado númerico. Além disso, é possível visualizar a transmissão ao vivo da imagem capturada pelo robô, juntamente com a temperatura registrada pelo seu sensor.
Ademais, há o sensor que detecta o quão próximo o robô está de algum objeto e não permite sua movimentação. 

&emsp;Por último, mas não menos importante há o botão de parada de emergência, quando o usuário aperta a tecla Q é cortada a conexão com o robô e o mesmo para de se mover.


#### Histórico de dados

&emsp;A página de histórico de dados tem o objetivo de servir como uma maneira de traçar quem utilizou o robô e o horário que isso foi feito.  O objetivo é servir com um registro de quem utilizou e ter uma rastreabilidade desse uso. 


<div align="center"> 

![Histórico de Dados - Mockup](../../../../..\docs\static\img\sprint3\history.png)
Fonte: Elaborado pelo grupo Cannabot
</div>

## Considerações e próximos passos 
&emsp;Durante esta sprint o foco do grupo foi desenvolver a integração da movimentação do robô e transmissão de imagem. Assim, outras partes do código não puderam ser feitas. 

&emsp;Na próxima sprint o objetivo é implementar o sensor de temperatura e realizar as integrações de histórico de usos. Os dados que o usuário está vendo no momento não são dinãmicos. 
