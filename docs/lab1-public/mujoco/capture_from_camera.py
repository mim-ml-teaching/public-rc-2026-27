import cv2
import mujoco


def main():
    model = mujoco.MjModel.from_xml_path("world1.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=480, width=640)
    renderer.update_scene(data, camera="main_camera")
    image = renderer.render()

    cv2.imwrite("main_camera.png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))


if __name__ == "__main__":
    main()
