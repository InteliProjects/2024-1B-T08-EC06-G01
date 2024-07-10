---
title: CLI
sidebar_position: 1
---

# CLI: Command-Line Interface

## Introdução
&emsp; Nessa secção será discustido a Interface de Linha de Comando e todos os processos que estão envolvidos com a mesma. Isso significa a movimentação do robô e o sistema de emergência por trás. 

### O que é 'CLI' afinal?

&emsp;A CLI, ou Interface de Linha de Comando, é uma área de interação simplificada que permite aos usuários interagirem com um software através de comandos de texto. Geralmente integrada no início do desenvolvimento de um projeto, ela é crucial para testar e validar funcionalidades de maneira simplificada. Sua importância se torna evidente à medida que o projeto evolui e se integra a uma aplicação frontend no futuro.

&emsp;A construção da CLI para o projeto do grupo Cannabot foi concebida com foco na simplicidade e intuitividade para os usuários que irão testar a aplicação nesta fase inicial. Consideramos que, sendo um passo preliminar antes da integração com a aplicação frontend, não era necessário investir esforços excessivos em sua construção. Abaixo, apresentamos a primeira versão da interface de linha de comando do grupo Cannabot e detalhamos o processo de sua construção a partir de seu próprio código:

### Interface

&emsp;Ao iniciar o pacote ROS, uma interface gráfica é exibida no terminal, permitindo a interação com o robô. A CLI apresenta um menu interativo onde os usuários podem selecionar a ação desejada para o robô. As opções disponíveis são:

- `front`: Move o robô para frente.
- `back`: Move o robô para trás.
- `left`: Move o robô para a esquerda.
- `right`: Move o robô para a direita.
- `stop`: Ativa a parada de emergência, interrompendo imediatamente todas as operações do robô.
- `exit`: Sai da CLI.

:::warning
`Se em qualquer momento você desejar parar o robô, pressione 'Q'`

A tecla "Q" é designada como atalho para ativar a parada de emergência. Este sistema de segurança é essencial para interromper imediatamente todas as operações do robô em caso de emergência ou mau funcionamento. Isso garante a segurança do ambiente e do robô, evitando danos ou acidentes.
::: 

<p align="center">Figura Command-Line Interface 1:</p>
<div align="center">
  ![CLI](../../../../static/img/sprint2/cli-certa.png)
  <p><b>Fonte:</b> Elaborado por Cannabot</p>
</div>

### Código

&emsp;Ao executar o pacote com o comando ros2 run cannabot cannabot no terminal, na verdade estamos chamando o arquivo cannabot.py e executando sua função principal main() a partir do seguinte trecho de código:

```python
if __name__ == "__main__":
    main()
```

&emsp;Ao executar o pacote com o comando `ros2 run cannabot cannabot` no terminal, a função principal `main()` é acionada, iniciando a CLI: 

```python
def show_menu():
    questions = [
        {
            'type': 'list',
            'name': 'action',
            'message': 'What do you want to do?',
            'choices': ['front', 'back', 'left', 'right', 'exit', 'stop'],
        },
    ]

    keybindings: InquirerPyKeybindings = {
        "interrupt": [{"key": "q"}, {"key": "c-c"}],
    }

    try:
        return prompt(questions, keybindings=keybindings)['action']
    except KeyboardInterrupt:
        return 'panic'
```

&emsp;Quando o usuário interage com nossa CLI e escolhe um comando para executar, a parte do código que interage com a classe do robô é acionada, resultando na movimentação correspondente. Por exemplo, se o usuário escolher "front", a classe do robô será chamada e ele executará a movimentação. A interação com o usuário é facilitada pela função `show_menu()`, onde as escolhas do usuário são processadas e as ações correspondentes do robô são executadas.

```python 
def main():
    rclpy.init(args=None)
    robot = TurtleBot()

    print(
"""
Se em qualquer momento você desejar parar o robô, pressione 'Q'.
"""
    )
    while True:
        action = show_menu()
        match action:
            case 'front':
                print("Mover para frente")
                robot.move_forward(0.1, 1.0)
            case 'back':
                print("Mover para trás")
                robot.move_backward(0.1, 1.0)
            case 'left':
                print("Mover para a esquerda")
                robot.rotate_left(2.0, 1.0)
            case 'right':
                print("Mover para a direita")
                robot.rotate_right(2.0, 1.0)
            case 'stop':
                print("Parada de emergência")
                robot.emergency_stop()
            case 'panic':
                print("Parada de emergência")
                robot.emergency_stop()
                robot.destroy_node()
                rclpy.shutdown()
                exit()
            case 'exit':
                robot.destroy_node()
                rclpy.shutdown()
                exit()
```

### Conclusão

&emsp;A CLI desempenha um papel fundamental no processo de desenvolvimento do projeto Cannabot, oferecendo uma maneira simples e direta para os usuários interagirem com a solução. Através dela, é possível testar e validar as funcionalidades do robô de forma eficiente, contribuindo para a evolução do projeto. Embora seja uma etapa preliminar antes da integração com a aplicação frontend, a CLI foi projetada com atenção aos detalhes, garantindo uma experiência intuitiva para os usuários. À medida que o projeto avança, a CLI continuará a ser aprimorada e integrada às demais partes da aplicação, impulsionando o desenvolvimento do Cannabot como um todo.





