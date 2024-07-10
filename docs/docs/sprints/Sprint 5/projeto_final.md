---
title: "Resumo das atividades desenvolvidas"
sidebar_position: 1
description: Nessa secção será detalhado tudo o que foi realizado durante o projeto.
---

## Introdução

&emsp;O sexto módulo do curso de Engenharia da Computação do Inteli tem o objetivo de ensinar aos alunos robótica móvel e visão computacional. Em parceria com a Atvos, uma empresa do ramo de bioenergia, surgiu um desafio que se encaixa no escopo do módulo: desenvolver um robô capaz de identificar sujeiras nos tubos do reboiler utilizando visão computacional. O reboiler é uma peça utilizada para aquecer o líquido que sai após a moagem da cana e seguir todas as etapas do processo na geração de bioenergia.

&emsp;Após um entendimento mais profundo do problema e uma entrevista com os responsáveis técnicos da Atvos, constatou-se que os reboilers são fundamentais para o funcionamento da fábrica, e a limpeza dos mesmos é feita semanalmente. Dessa forma, o grupo pensou em desenvolver um robô teleoperado que ficaria funcionando dentro do reboiler, coletando a temperatura interna e utilizando um sistema de visão computacional capaz de identificar objetos estranhos dentro do reboiler.

## Entendimento do problema

&emsp;O primeiro passo para desenvolver um software é entender quem são os usuários e quais são suas necessidades. Além disso, é preciso compreender como a Atvos está posicionada no mercado, quais são os riscos do projeto e realizar uma análise de viabilidade financeira.

Assim, durante a primeira sprint foram desenvolvidos os artefatos de [análise de risco](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%201/negocios/matriz), [bussiness model canvas](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%201/negocios/bmc), [personas](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%201/ux/personas), [requisitos do sistema](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%201/arquitetura/requisitos) e [arquitetura da informação](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%201/ux/arquitetura_info). 
Cada um desses artefatos tem o objetivo de entender melhor o problema, identificar quem vai utilizar a solução e detalhar como o grupo pensou em resolver o problema.

Com a primeira sprint concluída, já é possível avançar para o desenvolvimento do projeto. Agora está claro para o grupo quem é o usuário da solução, qual foi o fluxo da solução imaginado e como as partes se encaixam para gerar uma solução que faça sentido no final.


## Soluções desenvolvidas

&emsp;O processo de desenvolvimento de software é iterativo, permitindo mudanças incrementais e a flexibilidade de pivotar conforme as demandas do parceiro e um melhor entendimento do problema. Além disso, é possível ir melhorando cada componente individual do projeto ao longo das sprints.

&emsp;Desta forma, durante a segunda sprint a ênfase do grupo foi desenvolver um [prótotipo de interface](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%202/ux/interface), atualizar a [arquitetura](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%202/arquitetura/) com base em um melhor entendimento do projeto e aprender a utilizar o ecosistema do robô o [ROS](https://docs.ros.org/en/humble/index.html). 

O sistema de utilização do robô que foi desenvolvido durante esta etapa do projeto é um belo exemplo de como o desenvolvimento de software é dinâmico. O sistema de controle por [CLI](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%202/sistema-de-emergencia/teleoperacao) só serviu como base para uma implementação futura direto no frontend. 

&emsp;Já na terceira sprint o grupo teve muitas tarefas para realizar. Foi feito um melhor entendimento das finanças do projeto. Assim, foi desenvolvido uma [análise financeira](https://inteli-college.github.io/2024-1B-T08-EC06-G01/category/an%C3%A1lise-financeira) tendo em vista tanto os custos envolvidos em criar uma prova de conceito do zero e também os custos de implementar o robô em produção. 

Durante a terceira sprint foi dado o início no desenvolvimento do [frontend](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%203/Interface%20Usu%C3%A1rio/interface) e [backend](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%203/Backend/). Além disso, nessa sprint o grupo já foi capaz de conectar uma câmera ao robô,realizar a operação do mesmo pelo frontend do projeto, implementação de um sistema de emergência e detecção de objetos que possam impedir a movimentação do robô. O trabalho pode ser conferido no seguinte [link](https://inteli-college.github.io/2024-1B-T08-EC06-G01/category/configura%C3%A7%C3%B5es-do-rob%C3%B4)

&emsp; Na quarta sprint, o objetivo principal era implementar um modelo de [visão computacional](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%204/ia/modelo) que fosse capaz de ajudar o operador do robô a detectar objetos estranhos dentro do reboiler e realizar [testes com usuários reais](https://inteli-college.github.io/2024-1B-T08-EC06-G01/category/testes). 

Além disso, outra parte importante do projeto foi a integração do robô com o [sensor de temperatura](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%204/Sensor/), visto que este é primordial para a geração de dados do projeto e entender como está a situação dentro do reboiler. 

Na quinta sprint o objetivo é finalizar o projeto e ajeitar alguns detalhes que faltam para uma entrega bem feita.  
Na lista abaixo é possível clicar e ter acesso a versão final do projeto de cada uma das partes que o compõem. 

- [Arquitetura](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%205/projeto_final/arquitetura)
- [Como utilizar](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%205/projeto_final/como_util)
- [Frontend](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%205/projeto_final/frontend)
- [Backend](https://inteli-college.github.io/2024-1B-T08-EC06-G01/category/backend-2)
- [Inteligência Artificial](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%205/projeto_final/ia)
- [Operação do robô](https://inteli-college.github.io/2024-1B-T08-EC06-G01/category/opera%C3%A7%C3%A3o-do-rob%C3%B4)



Portanto, é possível perceber que ao longo das sprints muitas coisas foram mudando e se adaptando para que, no final, a entrega do projeto esteja funcionando de acordo com as especificações do parceiro.

## Conclusões 

&emsp;Durante as últimas 10 semanas, o grupo Cannabot atuou no desenvolvimento de um robô teleoperado para a medição de temperatura e detecção de anomalias dentro de um reboiler, em parceria com a Atvos.

No vídeo abaixo é possível conferir uma demonstração do robô em funcionamento na sua versão mais atual. [![Demonstração](https://img.youtube.com/vi/muJQy38vtec/0.jpg)](https://www.youtube.com/watch?v=muJQy38vtec)


Além disso, foi criada uma seção de [próximos passos](https://inteli-college.github.io/2024-1B-T08-EC06-G01/sprints/Sprint%205/projeto_final/proximos), pensada para os funcionários da Atvos que irão continuar o desenvolvimento da solução. Esta seção contém sugestões de features que podem ser implementadas, mas que não fizeram parte do escopo inicial ou não houve tempo suficiente para implementação.


