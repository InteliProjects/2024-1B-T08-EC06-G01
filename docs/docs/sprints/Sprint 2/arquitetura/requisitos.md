---
title: Atualizações nos Requisitos do Sistema
sidebar_position: 2
---

## Introdução
&emsp;Tendo em mente as mudanças que foram realizadas na arquitetura do projeto é preciso atualizar os requisitos funcionais e não funcionais do projeto. 
Os requisitos geralmente são classificados em duas categorias principais:

- Requisitos funcionais: são as funcionalidades específicas que o sistema deve oferecer para realizar as operações desejadas.

- Requisitos não funcionais: são as características de qualidade essenciais para o sistema. Eles asseguram que a solução atenda aos critérios de eficiência e eficácia, proporcionando uma base sólida para alcançar os requisitos funcionais com excelência.

Os requisitos que sofreram mudanças da sprint passada para a próxima foram: **RF1**,**RF2**,**RF4** e **RNF1**

# Atualizações nos Requisitos do Sistema

## Requisitos Funcionais

**RF1 - Algoritmo de varredura autônoma**

- A solução deve integrar um robô Turtlebot3, configurado para deslocar-se de forma autônoma dentro do reboiler, assegurando uma varredura completa da área.


**RF2 - Medição de Temperatura**

- O sistema deve utilizar um sensor de temperatura acoplado ao robô Turtlebot3 para coletar as medições de temperatura dentro do reboiler.

**RF3 - Interface Gráfica**

- A interface gráfica deve apresentar de forma clara o status atual do robô, indicando se está ligado, desligado ou enfrentando problemas técnicos, por meio de indicadores visuais distintos.

- Deve ser disponibilizada uma visualização em tempo real das imagens capturadas pela câmera, permitindo que os operadores monitorem visualmente o processo de coleta de dados no reboiler.

**RF4 - Sistema de Contingência**

- O usuário deve ser capaz de controlar o robô por meio da interface em casos de problemas técnicos, podendo ligá-lo, desligá-lo e ajustar suas movimentações na área de operação.

- O usuário do sistema deve ser capaz de visualizar o ponto de vista do robô por meio de uma câmera para poder navegar pelo ambiente.

## Requisitos Não Funcionais

**RNF1- Controle do robô**

- A câmera que será usada para o usuário controlar o robô deve possuir uma resolução mínima de 1080p e ser capaz de capturar imagens detalhadas do ambiente, garantindo uma visualização nítida e clara para possibilitar segurança na operação do robô.

- Métrica associada: Espera-se que o usuário não bata nas paredes do reboiler durante a operação do robô ao menos em 90% das ocorrências.


**RNF2 - Latência na Transmissão de Dados**

- A transmissão dos dados e imagens capturadas pela câmera integrada ao robô deve ocorrer com mínima latência, garantindo uma visualização em tempo real e uma resposta ágil dos operadores.

- Métrica associada: A latência média de transmissão de dados não deve exceder 300ms, assegurando uma comunicação eficiente e rápida das informações.

**RNF3 - Usabilidade da Interface Gráfica**

- A interface gráfica deve ser projetada visando a facilidade de uso e compreensão, permitindo que os operadores compreendam rapidamente o estado do robô e do processo de inspeção.

- Métrica associada: A usabilidade da interface será avaliada com base no número de etapas (cliques) necessárias para concluir uma tarefa, limitado a no máximo 3 cliques. Além disso, espera-se que 90% das funcionalidades estejam disponíveis ou indicadas diretamente na tela inicial, facilitando o acesso às ferramentas necessárias.



## Conclusão 
&emsp;Tendo em mente a mudança do escopo do projeto, a atualização nos requisitos funcionais e não funcionais destacados acima é de suma importância para o desenvolvimento de uma solução segura e eficaz. As principais alterações ocorreram nos requisitos funcionais, uma vez que o robô não irá mais verificar o estado de cada tubo do reboiler. Para mais explicações sobre as alterações, consultar a seção *Segunda Sprint*. 





