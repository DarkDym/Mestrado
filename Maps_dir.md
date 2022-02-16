# Modificação do caminho dos mapas

Durante as implementações do projeto do Jaguar utilizamos caminhos absolutos da máquina de cada membro da equipe para realizar o carregamento dos mapas necessários para o funcionamento do Jaguar em simulação, contudo no Jaguar o caminho absoluto para estes mapas é diferente do que utilizamos até o momento.

Sendo assim, este tutorial visa a modificação dos caminhos para os mapas que o Jaguar necessita para o seu funcionamento.

## Requisitos

- Terminator ou qualquer outro tipo de terminal
- Mapas utilizados no Jaguar, se encontram no caminho "CAMINHO_DO_WORKSPACE/src/instor/simulations/worlds/"

## Passo a Passo

- 1º Passo: Abrir um terminal.
- 2º Passo: Realizar o seguinte comando para criar o caminho da pasta onde serão colocados os mapas:

    ```
    sudo mkdir -p /mnt/persistent/instor
    ```

- 3º Passo: Realizar a cópia das imagens dos mapas e seus respectivos arquivos de configurações utilizando o comando:

    ```
    sudo cp /CAMINHO_PARA_OS_MAPAS_DENTRO_DO_FRAMEWORK /mnt/persistent/instor
    ```
    Obs.: Realizar o (3º Passo) para os seguintes arquivos:
    > - map.yaml
    > - map.pgm
    > - lane_map.yaml
    > - lane_map.pgm
    > - speed_map.yaml
    > - speed_map.pgm

