---
title: "Próximos Passos"
sidebar_position: 3
description: Esta secção tem como objeto sugerir formas de escalonar a solução desenvolvida pelo Cannabot.
---


## Próximos Passos

&emsp;A solução desenvolvida pelo grupo Cannabot, não é nada mais nada menos que uma Prova de Conceito (PoC), ou seja, uma TRL3 (Technology Readiness Level), que é uma demonstração prática que verifica a viabilidade e o potencial de um projeto em um contexto específico, tendo como objetivo comprovar que um conceito ou teoria pode ser implementado e funcionará conforme o esperado.

&emsp;Os Technology Readiness Levels (TRLs) são uma metodologia usada para medir a maturidade de uma tecnologia durante seu desenvolvimento. Eles variam de TRL1, que representa a observação inicial de princípios básicos, até TRL9, que indica que a tecnologia foi testada e está comprovadamente funcionando em um ambiente operacional real. Cada nível subsequente (de TRL2 a TRL8) representa estágios progressivos de desenvolvimento e validação, culminando na total implementação e uso comercial.

&emsp;Desse modo, a solução desenvolvida não sana todas as dores trazidas pelo parceiro, a Atvos. Sendo assim, o grupo Cannabot redigiu este documento a fim de sugerir formas de escalonar a solução, aprimorando-a para que, então, se transforme em uma TRL9.


### Substituição

&emsp;Como descrito, a solução desenvolvida se trata de uma Prova de Conceito, sendo assim, o material utilizado não é o mais apropriado para uma implementação além da TRL3. Sendo assim, durante a Sprint 3, foi desenvolvida uma Análise Financeira Final do projeto, considerando outro hardware que atendesse às necessidades específicas para resolver o problema trazido.

&emsp;O hardware indicado para substituir o Turtlebot 3 é o Spot Robot, um robô quadrúpede com uma ampla gama de aplicações industriais, pois possui uma capacidade de navegar em terrenos irregulares, suportando variedades como lama, areia e cascalho; também inclui pontos de montagem modulares fácil para integração de acessórios e sensores; também, pode ser controlado remotamente via uma interface do usuário. O robô já possui uma câmera 360° para navegação e captura de dados, além de sensores de detecção de obstáculos para evitar colisões e melhorar a segurança durante a operação.

&emsp;O Spot Robot oferece APIs abertas, permitindo a customização e programação do robô para tarefas específicas, suportando, também, integração com sistemas de software já existentes e desenvolvimento de aplicativos personalizados. Ademais, o hardware é construído para operar em ambientes diversos, sendo assim, é um robô resistente perante a tarefa proposta, tendo, também, a possibilidade de aprimorar sua bateria, como por exemplo, o uso de sistemas de bateria modular que permitem a substituição rápida e fácil das baterias, minimizando o tempo de inatividade.


### Treinamento do Modelo com outra Base de Dados:

&emsp;Necessitando de um modelo de visão computacional rápido na detecção de objetos estranhos dentro do reboiler, foi utilizado o YOLO V8, modelo é amplamente utilizado por diversas pessoas e empresas ao redor do mundo. Ele possui uma extensa documentação e pode ser aplicado em diversos tipos de problemas que requerem visão computacional.

&emsp;Pensando no problema de detectar objetos que possam estar dentro do reboiler - ambiente escuro e inóspito. O modelo atual utiliza um dataset que serve para identificar corpos estranhos em tubulações de esgoto.

&emsp;Pensando em melhorar o desempenho do modelo, recomenda-se a substituição do dataset atual, sendo interessante utilizar fotos de objetos dentro dos tubos tiradas pelos próprios funcionários da Atvos para gerar um novo dataset.


### Implementação a Nível Industrial:

&emsp;Durante o desenvolvimento da Prova de Conceito (PoC), a solução foi testada em um ambiente controlado, que não reflete completamente as condições e desafios de um ambiente industrial. Para alcançar o Technology Readiness Level (TRL) 9 e garantir que a solução seja robusta, eficiente e confiável em uma aplicação industrial real, deve-se considerar os seguintes passos:

1. Ambiente de Teste Industrial:

    a. É estritamente necessário estabelecer um ambiente de teste que simula as condições reais da indústria, incluindo a presença de poeira, umidade, temperatura extrema, além de outros fatores que podem impactar a performance do robô;

    b. Também, testes de durabilidade e resistência do hardware devem ser realizados em cenários prolongados e adversos, assegurando a longevidade e eficiência da solução.

2. Integração e Interoperabilidade:

    a. A integração deve garantir que o robô e o sistema de backend sejam totalmente compatíveis e integrados com o sistemas já utilizados pela Atvos;

    b. Ademais, implementar protocolos de comunicação padrão que permitam interoperabilidade entre diferentes sistemas e dispositivos na planta industrial.

3. Segurança e Confiabilidade:

    a. É importante fortalecer a segurança cibernética, incluindo criptografia dos dados, autenticação robusta e proteção contra ataques cibernéticos;

    b. Também, é estritamente essencial configurar um sistema de redundância e backup, garantindo a continuidade da operação em caso de falha.

4. Treinamento:
    a. Torna-se necessário especializar a equipe que irá operar a solução, garantindo que eles possam operar e manter a solução de maneira eficiente.
