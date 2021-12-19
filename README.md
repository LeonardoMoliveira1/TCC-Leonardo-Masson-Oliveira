# TCC-Leonardo-Masson-Oliveira
Trabalho de Conclusão de Curso - PANORAMA E PERSPECTIVAS FUTURAS DOS PRINCIPAIS SOFTWARES DE SIMULAÇÃO PARA ROBÓTICA MÓVEL: ESTUDO DE CASO COM COPPELIA ROBOTICS.

Esses códigos foram desenvolvidos considerando as últimas versões do Coppelia Robotics disponíveis em 2021. É possível que alguns comandos e bibliotecas sejam atualizados com o tempo, tornando necessária a manutenção do código.

Para o desenvolvimento dos estudos, foi o escolhido o Spyder, que é um ambiente de desenvolvimento integrado de plataforma cruzada de código aberto para programação científica na linguagem Python. A versão utilizada do Spyder no desenvolvimento foi a 3.8.

Para realizar a simulação em Coppelia Robotics com API em Python é necessário ter nas pastas em que constam os códigos os seguintes itens:

sim e simConst: bibliotecas para programação em ambiente Python.

remoteApi.dll: responsável por realizar a conexão servidor-client.

grid_with_orientation: imagem .PNG utilizada como referências para os algorítimos de visão computacional.

Acompanham, no arquivo, dois algorítimos:

(i) Evita.Colisao: código desenvolvido com os princípios de Braitenberg, com o propósito de evitar os obstáculos da cena afastando-se dos obstáculos mais próximos.
* Cena: cena_evita_colisao
- Modo Padrão: atribui os valores de esterçamento, velocidade inicial e tempo de simulação pré-definidos.
- Modo Manual: habilita ao usuário atribuir esses valores ao modelo.

(ii) Follow_the_Carrot: Esse algorítimo conta com visão computacional, algorítimos de planejamento de trajetórias e sistemas de controle  integrados para realizar os percursos determinados.
* Cena: Astar
