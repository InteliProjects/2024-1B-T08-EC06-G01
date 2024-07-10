---
title: "Testes dos Requisitos Não-Funcionais"
position: 2
---

&emsp;Este presente documento tem como objetivo detalhar os procedimenos e metodologias utilizados para a realização de testes de requisitps não funcionais da solução desenvolvida pelo grupo Cannabot. Os testes de requisitosnão funcionasi sçao essenciais para assegurar que o sistema atenta às expectativas de desempenhos, usabilidade e segurança, além de outros critérios de qualidade.

&emsp;Os requisitos analisados foram:

- **RNF2 - Desempenho do Modelo Preditivo:**
O modelo de aprendizado de máquina deve demonstrar alto desempenho na identificação de acúmulo de açúcar nos tubos, assegurando resultados precisos e confiáveis.

    *Métrica associada:* A métrica definida para garantir que o modelo funcione como o esperado é que:
    - a precisão seja de no mínimo 80%
    - o recall seja de no mínimo 80%
    - o F1-score* seja de no mínimo 0,85 (sendo 1,0 o máximo).

    *O F1-score é uma métrica que tem em seu cálculo a precisão (proporção de verdadeiros positivos em relação a todas as amostras identificadas como positivas) e o recall (proporção de verdadeiros positivos em relação a todas as amostras positivas reais). Será útil para garantir que não haja grande desbalanceamento entre classes.

    *Teste de validação*: para garantir que as métricas estipuladas estão sendo atingidas, após o treinamento do modelo de classificação, foi possível realizar o cálculo das métricas, obtendo o seguinte resultado:
    [Resultado das métricas]

    *Resultado do teste:* Como ilustrado anteriormente, embora a maioria das classes demonstre um desempenho satisfatório, a classe "dirt", que é crucial para a problemática do projeto, registra uma precisão de 92%, atendendo à métrica predefinida. No entanto, o recall de 40% é significativamente inferior ao esperado. Além disso, ao calcular o F1_score com base na precisão e recall obtidos, observamos que o resultado é aproximadamente 0,55, abaixo das expectativas.

    &emsp;O recall abaixo do esperado pode ter várias razões, incluindo o desbalanceamento de dados, onde a classe "dirt" pode estar sub-representada no conjunto de treinamento, resultando em um desempenho inferior no recall. Outra possível causa é a complexidade inerente ao problema, com a classe "dirt" sendo naturalmente mais desafiadora para o modelo identificar, exigindo ajustes específicos para alcançar as metas.

    &emsp;Para assegurar um melhor desempenho em relação às métricas estabelecidas, é crucial investigar profundamente os motivos por trás do baixo recall e, consequentemente, do F1_score associado à classe "dirt". Além disso, estamos considerando a transição do modelo atual para um modelo de classificação, adaptando-se melhor à problemática do projeto do que um modelo de detecção, o que poderá ter um impacto significativo nas métricas finais. Essas medidas visam melhorar a capacidade do modelo de identificar corretamente a classe se há ou não impurezas no reboiller e, assim, aprimorar sua eficácia na resolução do problema em questão.
    

- **RNF3 - Latência na Transmissão de Dados:**
A transmissão dos dados e imagens capturadas pela câmera integrada ao robô deve occorrer com mínima latência, garantinfo uma visualização em tempo real e uma resposta ágil dos operados.

    *Métrica:* A latência média de transmissão de dados não deve exceder 300ms, assegurando uma comunicação eficiente e rápida das informações.

    *Resultado dos Testes:* Durante os testes de uso, foram conduziadas avaliações com quatro usuários distintos, permitindo avaliar a latência durante a transmissão de dados e como isso afeta a solução.

    &emsp;Os resultados dos testes indicaram que apenas um dos quatro usuários não teve problemas significativos com latência, mantendo-se entre 400 e 500ms, enquantos os demais usuários enfrentaram problemas significativos com latência ainda maiores, comprometendo a eficiência da comunciação e a capacidade de respsota em tempo real, fazendo com que a teleoperação fosse prejudicada, ou até mesmo, incapaz de ser executada devido ao delay de transmissão.

    &emsp;Sendo assim, recomenda-se que seja feito uma revisão na configuração de rede para identificar possíveis gargalos ou problemas de infraestrutura que possam estar contribuindo para a alta latência. Também, recomenda-se avaliar o desemprenho do software de transmissão de dados e imagens para identificar possíveis melhorias e otimizações. Além disso, realizar testes adicionais e com um grupo maior de usuário e em diferentes condições de rede, pode fornecer uma visão mais abrangente dos problemas de latência.

- **RNF4 - Usabilidade da Interface Gráfica**
A interface gráfica deve ser projetada visando a facilidade de uso e compreensão, permitindo que os operadores compreendam rapidamente o estado do robô e do processo de inspeção.

    *Métrica associada:* A usabilidade da interface será avaliada com base no número de etapas (cliques) necessárias para concluir uma tarefa, limitado a no máximo 3 cliques. Além disso, espera-se que 90% das funcionalidades estejam disponíveis ou indicadas diretamente na tela inicial, facilitando o acesso às ferramentas necessárias.

    *Teste de validação:* Para garantir que a métrica atribuída ao RNF4 está sendo atendida como o esperado, foi elaborado o seguinte vídeo, que perpassa por toda estrutura do site e verifica se está dentro da métrica estipulada.

    [link do vídeo de teste de validação RNF4](https://drive.google.com/file/d/1j33O3B4T6b6pia0bMs6KvHA8peVNWGZc/view?usp=sharing)

    *Resultado do teste:* O vídeo de validação demonstra que a interface gráfica atende às métricas propostas, ou seja, as funcionalidades do sistema são facilmente acessíveis. No entanto, durante o teste de requisito funcional realizado com os usuários, foram identificados alguns pontos de melhoria que não foram detectados pelas métricas estabelecidas no RNF4.

    &emsp;Um dos pontos foi a ativação de um pop-up quando o robô encontra uma barreira e para de se movimentar em direção a ela. Esse feedback é importante para que o usuário entenda o status do sistema e não confunda com um problema operacional do robô. A ausência desse sistema afetou negativamente a usabilidade.

    &emsp;Outro ponto de melhoria identificado foi a necessidade de esclarecer que as setas da interface não movimentam o robô ao serem clicadas. Para movimentar o robô, é necessário pressionar as setas do teclado ou as teclas A, W, S, D. Esse aspecto levou um dos usuários, que pertence à faixa etária do público-alvo, a demorar para entender o modo de uso, indicando a importância de prestar atenção a esse ponto.

    &emsp;Dessa forma, percebe-se que, embora as métricas objetivas sejam úteis para avaliar a usabilidade da interface, os testes com usuários oferecem uma análise mais detalhada e abrangente sobre os aspectos que precisam ser melhorados. Esses testes permitem identificar nuances que podem passar despercebidas nas métricas quantitativas, mesmo considerando a possibilidade de vieses dos usuários. Portanto, a combinação de métricas objetivas com testes com usuários aumenta significativamente a probabilidade de identificar e implementar com sucesso as melhorias necessárias.