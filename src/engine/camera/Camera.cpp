#include "Camera.h"
#include <GLFW/glfw3.h>
#include <iostream>

using namespace engine::math;

Camera::Camera(Vec3f startPos)
    : position(startPos), worldUp(0, 1, 0)
{
    updateVectors();
}

void Camera::updateVectors() {
    Vec3f dir;
    dir.x = cos(degToRad(yaw)) * cos(degToRad(pitch));
    dir.y = sin(degToRad(pitch));
    dir.z = sin(degToRad(yaw)) * cos(degToRad(pitch));
    front = normalize(dir);

    right = normalize(cross(front, worldUp));
    up = normalize(cross(right, front));
}

void Camera::setWorldSpaceFrustom(const engine::math::Mat4f& vpMat)
{
    _frustom_WorldSpace = _frustom.getPlanes(vpMat);
}

void Camera::update(float deltaTime, GLFWwindow* window) {
    float velocity = movementSpeed * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position = position + front * velocity;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position = position - front * velocity;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position = position + right * velocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position = position - right * velocity;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        position = position + worldUp * velocity;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        position = position - worldUp * velocity;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window,true);
    updateVectors(); // always update basis vectors
}

Mat4f Camera::getViewMatrix() const {
    return lookAt(position, position + front, up); 
}
