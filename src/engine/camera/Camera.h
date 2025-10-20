#pragma once
#include <engine/math/math.h>
#include <GLFW/glfw3.h>
#include <iomanip>
#include <engine/camera/frustom_culling/FrustumCulling.h>

class Camera {

private:

    engine::math::Vec3f position;
    engine::math::Vec3f front;
    engine::math::Vec3f right;
    engine::math::Vec3f up;
    engine::math::Vec3f worldUp;

    float pitch = 0.0f;
    float yaw = -90.0f; // facing -Z
    float movementSpeed = 0.001f;

    engine::frustum::Frustom _frustom;
    engine::utils::array<engine::math::Vec4f, 6> _frustom_WorldSpace;

public:
    Camera(engine::math::Vec3f startPos);

    void update(float deltaTime, GLFWwindow* window); // handles input and updates view matrix
    engine::math::Mat4f getViewMatrix() const;

    void setSpeed(float s) { movementSpeed = s; }

    void updateVectors();
    void setWorldSpaceFrustom(const engine::math::Mat4f& vpMat);
    engine::math::Vec4f getPos() { return { position.x,position.y ,position.z,1.0f }; };
    engine::utils::array<engine::math::Vec4f, 6> getFrumstPlanes() { return _frustom_WorldSpace;};

};
