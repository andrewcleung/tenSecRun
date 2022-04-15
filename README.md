# 10 Second Run
We (Andrew and Kenro) have re-created a video game called “10 Second Run” using C code on the DE1-SoC computer. It is an 2D platform obstacle course game, where the player runs and jumps while dodging obstacles, to reach the finish line. The catch however, is that there is a 10 second timer for each level - if the player cannot finish the level within 10 seconds, they lose the game.


## How to load the game on cpulator
1. Copy main.c
2. Visit cpulator.01xz.net
3. Choose ARM - DE1-SOC board
4. Choose C as your language
5. Paste the code
6. Run the program

## How to play:
1. The player clicks the input key push buttons to determine which level to play. 
2. Switch values changes levels 1 – 3 (note: make sure all switches other than SW0 and SW1 are turned off). 
3. Key 0 resets the currently set level. Key3 resets the game to the home screen.
4.  Keyboard inputs W, A, D moves the players right left and jump. Use this to get from the red start platform, to the blue finish platform. 

**Note:** If the timer reaches 0, then the game ends and you lose. If you hit a fireball or a spike, you also lose.

## Notable features: 
**I/O devices:**

**Software Polling:** Keyboard input

**Interrupts:** switches, keys, A9 Private timer Player physics, player mechanics, player movements Front end design

Data structure

Double buffering

3 Levels to play

Title screen, winning screen, losing screen
