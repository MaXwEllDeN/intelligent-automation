# Controle de posição um robô de tração diferencial

## TODO:
1. Determine (utilizando modelo uniciclo) os ganhos das leis de controle P e PD (incluindo termo derivativo filtrado [sKd(c/(s+c))]) "go to goal" para o caso de um robô P3DX; explicite as especificações de projeto, e.g., velocidade de navegação, pose inicial, pose final, velocidade máxima das juntas, critério de parada.

2. Implemente/teste uma simulação computacional do modelo uniciclo e teste leis de controle P e PD (termo derivativo filtrado [sKd(c/(s+c))]) "go to goal" projetadas na etapa (1). Gere gráficos e interprete qualitativa e quantitativamente os resultados obtidos.

3. Implemente/teste leis de controle P e PD (incluindo o filtro derivativo) "go to goal" projetada na etapa (1) e compare os resultados com aqueles obtidos na etapa (2). Gere gráficos e interprete qualitativa e quantitativamente os resultados obtidos.
Obs: Na implementação da etapa (3), utilize a linguagem de programação denominada Lua (http://www.coppeliarobotics.com/helpFiles/en/luaCrashCourse.htm).
