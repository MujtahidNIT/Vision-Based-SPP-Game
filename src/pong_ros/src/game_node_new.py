#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import pygame

# Game parameters
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

game_window_width = 800
game_window_height = 600
paddle_width = 100
paddle_height = 20

class PongGame:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((game_window_width, game_window_height))
        pygame.display.set_caption("ROS Pong")

        # Background color
        self.bg_color = (0, 128, 255)  # Sky blue background

        self.rect_x = game_window_width // 2 - paddle_width // 2
        self.rect_y = game_window_height - paddle_height - 10
        self.rect_change_x = 0

        self.ball_x = 50
        self.ball_y = 50
        self.ball_change_x = 5
        self.ball_change_y = 5

        self.score = 0

        # Game state variables
        self.game_started = False
        self.paused = False 

        # Font for text display
        self.font = pygame.font.SysFont('Calibri', 20, False, False)

    def draw_initial_screen(self):
        self.screen.fill(self.bg_color)  # Fill with background color

        # Draw title or instructions
        title_font = pygame.font.SysFont('Calibri', 40, True, False)
        title_text = title_font.render("ROS Pong by Mohammad Mujtahid!", True, WHITE)
        title_rect = title_text.get_rect()
        title_rect.center = (game_window_width // 2, game_window_height // 2)
        self.screen.blit(title_text, title_rect)

        # Draw "Press Any Key to Start" message
        start_font = pygame.font.SysFont('Calibri', 25, True, False)
        start_text = start_font.render("Press any key to start", True, WHITE)
        start_rect = start_text.get_rect()
        start_rect.center = (game_window_width // 2, game_window_height * 3 // 4)
        self.screen.blit(start_text, start_rect)

        pygame.display.update()
    
    def wait_for_any_key(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                # Check for any key press to start the game
                if event.type == pygame.KEYDOWN:
                    self.game_started = True
                    return True

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            # Handle starting, pausing, and unpausing based on key presses
            if event.type == pygame.KEYDOWN:
                if not self.game_started:
                    if event.key == pygame.K_SPACE:  # Change to desired key for starting
                        self.game_started = True
                elif self.paused:
                    if event.key == pygame.K_ESCAPE:  # Change to desired key for pausing
                        self.paused = False
                    elif event.key == pygame.K_SPACE:  # Change to desired key for resuming
                        self.paused = False
                else:
                    # Handle other gameplay-related key presses if needed
                    if event.key == pygame.K_LEFT:
                        self.rect_change_x = -5
                    elif event.key == pygame.K_RIGHT:
                        self.rect_change_x = 5

    
    def update(self):

        if self.game_started and not self.paused:
        # Update game logic if game has started and not paused

            self.rect_x += self.rect_change_x
            if self.rect_x <= 0:
                self.rect_x = 0
            elif self.rect_x >= game_window_width - paddle_width:
                self.rect_x = game_window_width - paddle_width

            self.ball_x += self.ball_change_x
            self.ball_y += self.ball_change_y

            # Handle ball collisions with walls
            if self.ball_x < 0 or self.ball_x > game_window_width - 15:
                self.ball_change_x *= -1
            if self.ball_y < 0:
                self.ball_change_y *= -1
            elif self.ball_y > game_window_height - 15:
                self.ball_change_y *= -1
                self.score = 0  # Reset score if ball hits bottom

            # Handle ball collision with paddle
            if self.ball_y == game_window_height - 15 - paddle_height and self.ball_x >= self.rect_x and self.ball_x <= self.rect_x + paddle_width:
                self.ball_change_y *= -1
                self.score += 1



        # self.rect_x += self.rect_change_x
        # if self.rect_x <= 0:
        #    self.rect_x = 0
        # elif self.rect_x >= game_window_width - paddle_width:
        #    self.rect_x = game_window_width - paddle_width

        # self.ball_x += self.ball_change_x
        # self.ball_y += self.ball_change_y

        # #Handle ball collisions with walls
        # if self.ball_x < 0 or self.ball_x > game_window_width - 15:
        #    self.ball_change_x *= -1
        # if self.ball_y < 0:
        #    self.ball_change_y *= -1
        # elif self.ball_y > game_window_height - 15:
        #    self.ball_change_y *= -1
        #    self.score = 0  # Reset score if ball hits bottom

        # #Handle ball collision with paddle
        # if self.ball_y == game_window_height - 15 - paddle_height and self.ball_x >= self.rect_x and self.ball_x <= self.rect_x + paddle_width:
        #    self.ball_change_y *= -1
        #    self.score += 1

    def render(self):



        self.screen.fill(self.bg_color)  # Fill with background color

        # Draw walls (optional, modify as desired)
        pygame.draw.rect(self.screen, WHITE, [0, 0, game_window_width, 5])  # Top wall
        pygame.draw.rect(self.screen, WHITE, [0, game_window_height - 5, game_window_width, 5])  # Bottom wall

        # Draw paddle
        pygame.draw.rect(self.screen, RED, [self.rect_x, self.rect_y, paddle_width, paddle_height])

        # Draw ball (replace with a better sprite if desired)
        pygame.draw.circle(self.screen, WHITE, [self.ball_x, self.ball_y], 10)

        # Display score
        score_text = self.font.render("Score: " + str(self.score), True, WHITE)
        self.screen.blit(score_text, [10, 10])

        # Display name
        name_text = self.font.render("Mohammad Mujtahid", True, WHITE)
        self.screen.blit(name_text, [game_window_width - name_text.get_width() - 10, 10])

        pygame.display.flip()



        # self.screen.fill(self.bg_color)  # Fill with background color

        # # Draw walls (optional, modify as desired)
        # pygame.draw.rect(self.screen, WHITE, [0, 0, game_window_width, 5])  # Top wall
        # pygame.draw.rect(self.screen, WHITE, [0, game_window_height - 5, game_window_width, 5])  # Bottom wall

        # # Draw paddle
        # pygame.draw.rect(self.screen, RED, [self.rect_x, self.rect_y, paddle_width, paddle_height])

        # # Draw ball (replace with a better sprite if desired)
        # pygame.draw.circle(self.screen, WHITE, [self.ball_x, self.ball_y], 10)

        # # Display score
        # score_text = self.font.render("Score: " + str(self.score), True, WHITE)
        # self.screen.blit(score_text, [10, 10])

        # # Display name
        # name_text = self.font.render("Mohammad Mujtahid", True, WHITE)
        # self.screen.blit(name_text, [game_window_width - name_text.get_width() - 10, 10])

        # pygame.display.flip()

def update_paddle(data):
    global game
    paddle_x = data.data * (game_window_width - paddle_width)
    game.rect_x = int(paddle_x)

def main():
    global game

    rospy.init_node('pong_game')

    game = PongGame()

    rospy.Subscriber('/paddle_position', Float32, update_paddle)

    rate = rospy.Rate(60)

    # Display the initial screen before entering the game loop
    game.draw_initial_screen()
    game.wait_for_any_key()
    print(game.game_started)

    while not rospy.is_shutdown():

        

        # Handle events in each iteration
        game.handle_events()


        if game.game_started and not game.paused:
            # Update and render the game only if started and not paused
            game.update()
            game.render()

        rate.sleep()


        # if game.game_started is False:
        #     game.draw_initial_screen()
        #     game.wait_for_any_key()
        # game.update()
        # game.render()
        # rate.sleep()

if __name__ == '__main__':
    main()

