
# Manual de Acesso ao Jaguar


## Ferramentas


- ### Requisitos Necessários


> Para a execução dos códigos do framework é necessário os seguintes itens:
>
> - **_NMAP Instalado_** 
> - **_IP_DO_JAGUAR_**
> - **_Terminator_**
> - **_Ubuntu 20_**
> - **_Framework e Acesso ao Git_**
> - **_Dentro do Terminator - 4 terminais abertos_**
> - **_Conexão com a rede Wi-Fi da Instor_**

    
- ### Recomendações

> - **_Dentro do Terminator - 6 terminais abertos_**


## Passo a Passo


### Passos necessários para a execução e conexão com o Jaguar:

> ### Para a execução dos códigos do framework é necessário os seguintes itens:
>
> - 1º Passo: Utilizar o comando '**nmap 10.0.0.0/24**' para descobrir o **IP_DO_JAGUAR**.
(Obs. Utilizando este comando irá aparecer todos os dispositivos conectados a rede, sendo assim a descoberta do **IP_DO_JAGUAR** não é trivial, os possíveis ips possuem portas específicas abertas, sendo elas: 22/tcp, 80/tcp e 443/tcp);
> - 2º Passo: Abrir o **Terminator** e dividir em no mínimo 4 terminais.
> - 3º Passo: Em todos os terminais abertos utilizar o comando "**cd WORKSPACE/**".
> - 4º Passo: Em todos os terminais abertos utilizar o comando "source devel/setup.bash" para adicionar as variáveis de ambiente para a pasta local e permitir que o ROS funcione.
> - 5º Passo: Em um dos terminais executar o comando "**cd WORKSPACE/scr/instor/dev-tools**".
(Obs. Em todos os terminais que irão fazer acesso direto ao Jaguar ou irão fazer acesso aos tópicos do Jaguar o "**5º Passo e o 6º Passo**" devem ser realizados).
> - 6º Passo: Nos terminais do "**5º Passo**" executar o comando "**source setup_env.bash -r **IP_DO_JAGUAR****" para fazer com que o ROS "se conecte" ao ROS do Jaguar.
> - 7º Passo: Em um terminal com o "**5º Passo e o 6º Passo**" realizado executar o comando "**ssh -i WORKSPACE/src/instor/instor-os/keys/instor-os-global root@IP_DO_JAGUAR**" para se conectar ao terminal do Jaguar por SSH.
> - 8º Passo: No terminal do "**7º Passo**" com o arquivo "**get-ip.bash**" aberto, modificar a última linha deste arquivo, onde possui o ip "**10.70.0.1**", e colocar o **IP_DO_JAGUAR** e salvar o arquivo.
> - 9º Passo: Em um terminal (diferente do "**7º Passo**") com o "**5º Passo e o 6º Passo**" realizado executar o comando "**./deploy_product.bash jaguar**" para compilar todo o código do framework e fazer o upload para o Jaguar.
> - 10º Passo: Em um terminal (diferente do "**7º Passo, do 8º Passo e do 9º Passo**") com o "**5º Passo e o 6º Passo**" realizado executar o comando "**rosrun rviz rviz**" para abrir o RViz.
> - 11º Passo: Na janela do Rviz aberta, carregar o arquivo de configurações "**instor_jaci.rviz**".
> - 12º Passo: No terminal do "**7º Passo**" executar o comando "**nano /opt/instor/get-ip.bash**" para configurar o IP que está sendo usado e permitir a visualização dos dados dos tópicos no RViz.
> - 13º Passo: Em um terminal (diferente do "**7º Passo e do 8º Passo**") com o "**5º Passo e o 6º Passo**" realizado executar o comando "**rostopic pub /operation_manager/request_str std_msgs/String "data: 'MODO_DE_OPERAÇÂO'" -1**" para definir o modo de operação (ex: idle, setup_localization, setup_docking, navigating, joystick).


## Comandos Úteis 

- ### "**journalctl -f**" - Comando para visualizar o que o Jaguar está executando, e verificar possíveis erros na execução dos serviços do Jaguar. (Obs. Este comando só pode ser executado dentro do ssh)
- ### "**journalctl -u instor*** **-f**" - Mesmo comando anterior, mas a diferença é que este aqui filtra todos os processos/serviços com o nome "instor***" (Obs. Este comando só pode ser executado dentro do ssh).
- ### "**systemctl --user stop instor*** && systemctl start instor-launcher". - Comando para desligar todos os serviços associados ao nome/usuário instor e executar o launcher do sistema da instor. Mesmo procedimento que desligar e ligar o Jaguar, porém mais rápido e fácil. (Obs. Este comando só pode ser executado dentro do ssh).
- ### "**scp -i WORKSPACE/src/instor/instor-os/keys/instor-os-global ARQUIVO_PARA_ENVIAR  root@IP_DO_JAGUAR/CAMINHO_DA_PASTA_DO_JAGUAR_PARA_RECEBER_O_ARQUIVO**" - Comando para enviar arquivos para dentro do Jaguar.
- ### "**watch df**" - Comando que serve para verificar o uso do HD.