---
title: Requisitos do Sistema
sidebar_position: 3
---

# Requisitos do Sistema

&emsp;Os requisitos de um sistema delineiam as funcionalidades essenciais que uma solução deve incluir para satisfazer as necessidades e expectativas dos usuários e partes interessadas, além de alcançar os objetivos estabelecidos pelos stakeholders.

&emsp;Esses requisitos são geralmente classificados em duas categorias principais:

- Requisitos funcionais: são as funcionalidades específicas que o sistema deve oferecer para realizar as operações desejadas.

- Requisitos não funcionais: são as características de qualidade essenciais para o sistema. Eles asseguram que a solução atenda aos critérios de eficiência e eficácia, proporcionando uma base sólida para alcançar os requisitos funcionais com excelência.

&emsp;Para o presente projeto, visando satisfazer as necessidades das personas identificadas e entregar o valor estipulado, a equipe elaborou os seguintes requisitos:

## Requisitos Funcionais

**RF1 - Algoritmo de varredura autônoma**

- A solução deve integrar um robô Turtlebot3, configurado para deslocar-se de forma autônoma pelos tubos do reboiler, assegurando uma varredura completa da área.

- Deve ser implementado um algoritmo preciso, capaz de identificar a localização exata de cada tubo, garantindo que as leituras sejam realizadas nos locais corretos.


**RF2 - Identificação de Impurezas**

- O sistema deve utilizar uma câmera de alta resolução acoplada ao robô Turtlebot3 para capturar imagens dos tubos dos reboilers.

- Deve ser elaborado um modelo preditivo, treinado com técnicas de aprendizado de máquina, para distinguir com precisão entre imagens que exibem impurezas e aquelas que não o apresentam.


**RF3 - Interface Gráfica**

- A interface gráfica deve apresentar de forma clara o status atual do robô, indicando se está ligado, desligado ou enfrentando problemas técnicos, por meio de indicadores visuais distintos.

- Deve ser disponibilizada uma visualização em tempo real das imagens capturadas pela câmera, permitindo que os operadores monitorem visualmente o processo de inspeção.

- A interface deve incluir uma contagem dinâmica dos tubos verificados até o momento, atualizando-se automaticamente à medida que o robô avança na inspeção.

- O usuário deve ser capaz de controlar o robô por meio da interface em casos de problemas técnicos, podendo ligá-lo, desligá-lo e ajustar suas movimentações pelo reboiler.

## Requisitos Não Funcionais

**RNF1- Qualidade da Detecção de tubos**

- A câmera deve possuir uma resolução mínima de 1080p e ser capaz de capturar imagens detalhadas dos tubos, garantindo uma visualização nítida e clara para possibilitar a leitura precisa de informações.

- Métrica associada: Espera-se que as imagens registradas possam capturar completamente a borda de cada tubo em pelo menos 90% das ocorrências.

**RNF2 - Desempenho do Modelo Preditivo**

- O modelo de aprendizado de máquina deve demonstrar alto desempenho na identificação de acúmulo de açúcar nos tubos, assegurando resultados precisos e confiáveis.

- Métrica associada: A métrica definida para garantir que o modelo funcione como o esperado é que:
    - a precisão seja de no mínimo 80%
    - o recall seja de no mínimo 80%
    - o F1-score* seja de no mínimo 0,85 (sendo 1,0 o máximo).

*O F1-score é uma métrica que tem em seu cálculo a precisão (proporção de verdadeiros positivos em relação a todas as amostras identificadas como positivas) e o recall (proporção de verdadeiros positivos em relação a todas as amostras positivas reais). Será útil para garantir que não haja grande desbalanceamento entre classes.

**RNF3 - Latência na Transmissão de Dados**

- A transmissão dos dados e imagens capturadas pela câmera integrada ao robô deve ocorrer com mínima latência, garantindo uma visualização em tempo real e uma resposta ágil dos operadores.

- Métrica associada: A latência média de transmissão de dados não deve exceder 300ms, assegurando uma comunicação eficiente e rápida das informações.

**RNF4 - Usabilidade da Interface Gráfica**

- A interface gráfica deve ser projetada visando a facilidade de uso e compreensão, permitindo que os operadores compreendam rapidamente o estado do robô e do processo de inspeção.

- Métrica associada: A usabilidade da interface será avaliada com base no número de etapas (cliques) necessárias para concluir uma tarefa, limitado a no máximo 3 cliques. Além disso, espera-se que 90% das funcionalidades estejam disponíveis ou indicadas diretamente na tela inicial, facilitando o acesso às ferramentas necessárias.


&emsp;Com base na cuidadosa definição dos requisitos funcionais e não funcionais destacados acima, nossa solução assegura a acessibilidade e eficácia tanto para Cleber, garantindo o funcionamento otimizado do robô e a validação da limpeza dos tubos, quanto para Ísis, proporcionando a capacidade de monitorar o processo de forma remota, visualizando dados correlacionados e intervindo conforme necessário. Esta abordagem abrangente garante uma experiência integrada e eficiente para todos os usuários envolvidos.





