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

        self.rect_x = game_window_width // 2 - paddle_width // 2
        self.rect_y = game_window_height - paddle_height - 10
        self.rect_change_x = 0

        self.ball_x = 50
        self.ball_y = 50
        self.ball_change_x = 5
        self.ball_change_y = 5

        self.score = 0

    def update(self):
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

    def render(self):
        self.screen.fill(BLACK)
        pygame.draw.rect(self.screen, RED, [self.rect_x, self.rect_y, paddle_width, paddle_height])
        pygame.draw.rect(self.screen, WHITE, [self.ball_x, self.ball_y, 15, 15])

        font = pygame.font.SysFont('Calibri', 15, False, False)
        text = font.render("Score = " + str(self.score), True, WHITE)
        self.screen.blit(text, [600, 100])

        pygame.display.flip()

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

    while not rospy.is_shutdown():
        game.update()
        game.render()
        rate.sleep()

if __name__ == '__main__':
    main()
