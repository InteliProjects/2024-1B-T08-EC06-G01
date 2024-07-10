---
title: "Atualização dos requisitos do sistema"
position: 1
---

&emsp;Os requisitos de um sistema delineiam as funcionalidades essenciais que uma solução deve incluir para satisfazer as necessidades e expectativas dos usuários e partes interessadas, além de alcançar os objetivos estabelecidos pelos stakeholders.

Esses requisitos são geralmente classificados em duas categorias principais:

- Requisitos funcionais: são as funcionalidades específicas que o sistema deve oferecer para realizar as operações desejadas.

- Requisitos não funcionais: são as características de qualidade essenciais para o sistema. Eles asseguram que a solução atenda aos critérios de eficiência e eficácia, proporcionando uma base sólida para alcançar os requisitos funcionais com excelência.

Na Sprint 1 foram definidos os requisitos funcionais e não funcionais do projeto, porém, ao longo do desenvolvimento surgiram alterações, que serão documentadas a seguir:

## Requisitos funcionais atualizados

**RF1 - Medição de Temperatura**

- O sistema deve utilizar um sensor de temperatura acoplado ao robô Turtlebot3 para coletar as medições de temperatura dentro do reboiler.

- Esses dados devem ser disponibilizados na interface gráfica em tempo real, e deve ser possível visualizar um heatmap com base nos dados coletados.

**RF2 - Identificação de Impurezas**

- O sistema deve utilizar uma câmera de alta resolução acoplada ao robô Turtlebot3 para capturar imagens dos tubos dos reboilers.

- A partir de um modelo e dataset adequados, em conjunto com o treinamento com técnicas de aprendizado de máquina, deve ser possível distinguir imagens que exibem impurezas e aquelas que não o apresentam, sendo possível acessar esses dados pela interface.

**RF3 - Interface Gráfica**

-  A interface gráfica deve possibilitar a teleoperação completa do robô, permitindo sua movimentação precisa pelo reboiler por meio de comandos intuitivos. Ademais, deve incluir um botão de emergência de fácil acesso para interromper imediatamente a operação do robô em caso de necessidade.

- A interface deve oferecer uma visualização em tempo real das imagens capturadas pela câmera do robô, permitindo que os operadores acompanhem a operação de forma contínua. Essa visualização deve ser clara e detalhada, garantindo que os operadores possam movimentar o robô com precisão por meio da interface gráfica.

## Requisitos não funcionais atualizados

**RNF1- Controle seguro do robô**

- A teleoperação do robô deve garantir que, ao controla-lo, o usuário seja impedido de colidir com objetos ou pessoas ao redor, mesmo que mova o robô em sua direção.

- Métrica associada: Espera-se que o usuário não bata nas paredes do reboiler durante a operação do robô ao menos em 90% das ocorrências.

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

&emsp;Após a revisão e atualização dos requisitos funcionais e não funcionais mencionados anteriormente, podemos assegurar que o projeto está alinhado com o escopo fornecido pelo parceiro. Além disso, é capaz de atender às suas necessidades estabelecidas de forma eficaz e eficiente.