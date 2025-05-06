import os, sys, json, argparse, subprocess
from distutils import util
from pathlib import Path
from pprint import pprint


def image_check_fn(image_name: str) -> bool:
    """
    Checks to see if the given image of format 'repo:tag' exists
    """
    split_image_name = image_name.split(":")
    repo = split_image_name[0]
    tag = split_image_name[1]
    image_check = subprocess.run(
        f"docker images --filter=reference='{image_name}'",
        shell=True,
        check=False,
        stdout=subprocess.PIPE,
    ).stdout
    return bool(tag in str(image_check)) and bool(repo in str(image_check))


def container_check_fn(container_name: str) -> bool:
    """
    Checks to see if the given container name is already in use exists
    """
    container_check = subprocess.run(
        f"docker container ls --filter=name='{container_name}'",
        shell=True,
        check=False,
        stdout=subprocess.PIPE,
    ).stdout
    return bool(container_name in str(container_check))


def main(args):
    ## Load the config
    with open(args["config"], "r") as f:
        config = json.load(f)
    print(f"Using config:")
    pprint(config)

    # nvidia-cuda... base image to build from
    base_image = config["base-image"]

    # construct volume mounts
    mounts = []
    for k, v in config["mounts"].items():
        mounts.append(f"{str(Path(k).absolute())}:{v}")

    ## Build the image if necessary
    image_check = image_check_fn(config["image-name"])
    if (not image_check) or (args["rebuild"] is True):
        print(f"\nBuilding {config['image-name']}...")
        built = True
        build_cmd = f"docker build \
                --build-arg BASEIMAGE={base_image} \
                -t {config['image-name']} \
                -f {config['dockerfile']} \
                ."
        subprocess.run(build_cmd, shell=True)
        print(f"Done building {config['image-name']}")
    else:
        print(f"\nSkipping build of {config['image-name']}")

    # Generate a container name that isn't taken
    n = 1
    container_name = config["container-name"]
    while container_check_fn(container_name):
        container_name = f"{container_name}-{n}"
        n += 1

    ## Run the container
    run_cmd = (
        f"rocker --mode interactive --name {container_name}"
        ' --net=host --env="DISPLAY" '
    )
    run_cmd += f'--volume "$HOME/.Xauthority:/root/.Xauthority:rw" '
    if args["device"] != "":
        run_cmd += f"--volume \"{args['device']}:{args['device']}\" "
        # run_cmd += f"--device {args['device']} "
    if args["headless"] is False:
        run_cmd += "--x11 "
    if args["cuda"] is True:
        run_cmd += "--cuda "
    if args["nvidia"] is True:
        run_cmd += "--nvidia "
    if len(mounts) > 0:
        run_cmd += "--volume "
        run_cmd += " ".join([f"{m}" for m in mounts])
        run_cmd += " -- "
    run_cmd += f"{config['image-name']}"
    print(f"\nRunning {container_name} with command...")
    pprint(run_cmd)
    print("Done.")
    os.system(run_cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", "--config", help="Path to config file", default="./desktop/config.json"
    )
    parser.add_argument(
        "--rebuild",
        help="Force rebuild of docker image",
        dest="rebuild",
        type=lambda x: bool(util.strtobool(x)),
    )
    parser.add_argument(
        "--headless",
        help="Run with or without an attached display.",
        dest="headless",
        type=lambda x: bool(util.strtobool(x)),
        default=False
    )
    parser.add_argument(
        "--cuda",
        help="Run with cuda",
        dest="cuda",
        type=lambda x: bool(util.strtobool(x)),
        default=False
    )
    parser.add_argument(
        "--nvidia",
        help="Run with nvidia",
        dest="nvidia",
        type=lambda x: bool(util.strtobool(x)),
        default=False
    )
    parser.add_argument(
        "-d", "--device", help="Add a serial device eg. /dev/tty0.", default=""
    )
    args = vars(parser.parse_args())
    main(args)
