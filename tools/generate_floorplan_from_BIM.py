#!/bin/env/python3

# This file is part of ivS-Graphs.
#
# Copyright (C) 2023-2025 SnT, University of Luxembourg
# Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
#
# ivS-Graphs is free software: you can redistribute it and/or modify it under the terms
# of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
#
# ivS-Graphs is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with this program.
# If not, see <https://www.gnu.org/licenses/>.

import os
import argparse
import cv2
import numpy as np


def parse_bim_file(bim_file):
    """
    Placeholder function to parse a BIM file.
    This function should be implemented to extract necessary data from the BIM file.
    """
    bim_data = []
    with open(bim_file, "r") as file:
        data = file.readlines()
        # first line is the header with the column names
        header = data[0].strip().split(",")
        # subsequent lines are the data Dependencies
        for line in data[1:]:
            wall_data = {}
            values = line.strip().split(",")
            for i, value in enumerate(values):
                wall_data[header[i].lower()] = value
            bim_data.append(wall_data)
    return bim_data


def compute_wall_lines(bim_data):
    """
    Compute wall lines from BIM data.
    This function should be implemented to convert BIM data into wall lines.
    """
    wall_lines = []
    for wall in bim_data:
        # each line starts in x-min, y-min, has a lenght, and a normal vector x-nor, y-nor
        x_min = float(wall["x-min"])
        y_min = float(wall["y-min"])
        length = float(wall["length"])
        x_nor = float(wall["x-nor"])
        y_nor = float(wall["y-nor"])
        # Calculate the end point of the wall line using the director vector as the perpendicular vector
        dir_vector = (y_nor, -x_nor)
        x_max = x_min + length * dir_vector[0]
        y_max = y_min + length * dir_vector[1]
        # if the end point is not in the first quadrant, flip the director vector
        if x_max < x_min or y_max < y_min:
            dir_vector = (-dir_vector[0], -dir_vector[1])
            x_max = x_min + length * dir_vector[0]
            y_max = y_min + length * dir_vector[1]

        wall_lines.append(((x_min, y_min), (x_max, y_max)))

    return wall_lines


def generate_floorplan_image(wall_lines, scale):
    """
    Generate a floorplan image from wall lines.
    This function should be implemented to create an image representation of the floorplan.
    """

    # compute the borders wiht a margin of 1m
    min_x = min(line[0][0] for line in wall_lines) - 1
    max_x = max(line[1][0] for line in wall_lines) + 1
    min_y = min(line[0][1] for line in wall_lines) - 1
    max_y = max(line[1][1] for line in wall_lines) + 1
    width = int((max_x - min_x) * scale)
    height = int((max_y - min_y) * scale)
    lines_positions = []

    floorplan_image = (
        np.ones((height, width, 3), dtype=np.uint8) * 255
    )  # white background
    for line in wall_lines:
        start = (int((line[0][0] - min_x) * scale), int((line[0][1] - min_y) * scale))
        end = (int((line[1][0] - min_x) * scale), int((line[1][1] - min_y) * scale))
        cv2.line(floorplan_image, start, end, (0, 0, 0), thickness=1)
        lines_positions.append((start, end))

    return floorplan_image, lines_positions


def flood_pixels(in_image, new_color):
    """
    Flood fill algorithm to colorize pixels in the image.
    Starts from the start point and colors all connected pixels with the same color as the start point.
    outputs a new image with only the filled pixels colored in the new_color.
    """
    image = (
        in_image.copy()
    )  # Work on a copy of the image to avoid modifying the original
    orig_image = image.copy()  # Keep a copy of the original image
    start_point = (0, 0)  # Starting point for flood fill
    start_color = image[
        start_point[1], start_point[0]
    ].tolist()  # Get the color of the starting pixel
    flood_mask = np.zeros((image.shape[0] + 2, image.shape[1] + 2), np.uint8)
    cv2.floodFill(
        image,
        flood_mask,
        start_point,
        new_color,
        loDiff=(10, 10, 10),
        upDiff=(10, 10, 10),
        flags=cv2.FLOODFILL_FIXED_RANGE,
    )
    filled_mask = flood_mask[1:-1, 1:-1]  # Remove the border added by floodFill
    # Create a new image to hold the filled pixels
    filled_image = np.zeros_like(image)
    # Color only the pixels that were filled
    filled_image[filled_mask == 1] = (
        255,
        255,
        255,
    )  # Set the color of the filled pixels to new_color
    # Combine the original image with the filled pixels
    # filled_image *= orig_image  # Keep the original image's colors
    inverted_image = cv2.bitwise_not(orig_image)  # Invert the original image
    filled_image = cv2.bitwise_or(
        filled_image, inverted_image
    )  # Combine the filled pixels with the inverted image
    # Colorize the filled pixels with the new color
    colorized_image = np.ones_like(image) * 255  # Start with a white image
    colorized_image[filled_mask == 0] = (
        new_color  # Set the color of the filled pixels to new_color
    )
    colorized_image = cv2.bitwise_and(
        orig_image, colorized_image
    )  # Combine the original image with the colorized filled pixels
    # cv2.imshow("Colorized Image", colorized_image)

    return filled_image, colorized_image


def write_permissive_space_from_flooded_image(flooded_image, base_filename):
    """
    Write the permissive space from the flooded image to a file.
    This function should be implemented to save the permissive space data.
    it will save a .txt file with the pixels that are part of the permissive space.
    """
    r_channel = flooded_image[:, :, 0]
    pixels = zip(
        *np.where(r_channel == 255)
    )  # Find all pixels in the red channel that are white (255
    print(f"pixels {pixels}")
    with open(f"{base_filename}_permissive_space.txt", "w") as file:
        for y, x in pixels:
            file.write(f"{int(x)} {int(y)}\n")  # Write the coordinates in x,y format
    print(f"Permissive space saved to {base_filename}_permissive_space.txt")


def write_walls_file(lines_positions, base_filename):
    """
    Write the wall lines to a file.
    This function should be implemented to save the wall lines data.
    use float coordinates in the format x1,y1,x2,y2
    """
    with open(f"{base_filename}_walls.txt", "w") as file:
        for start, end in lines_positions:
            # Convert the coordinates to float and format them
            x1, y1 = float(start[0]), float(start[1])
            x2, y2 = float(end[0]), float(end[1])
            file.write(f"{x1:e},{y1:e},{x2:e},{y2:e}\n")

    print(f"Walls saved to {base_filename}_walls.txt")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate a floorplan from a BIM file."
    )
    parser.add_argument("bim_file", type=str, help="Path to the BIM file.")
    parser.add_argument(
        "-r",
        "--resolution",
        type=float,
        default=0.1,
        help="Resolution of the output floorplan in meters per pixel.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="out",
        help="Output directory for the generated floorplan image. Defaults to current directory.",
    )

    args = parser.parse_args()

    # Check if the BIM file exists
    if not os.path.isfile(args.bim_file):
        raise FileNotFoundError(f"The BIM file {args.bim_file} does not exist.")

    # Placeholder for actual floorplan generation logic
    if args.output:
        make_output_dir = os.makedirs(args.output, exist_ok=True)

    base_name = os.path.basename(args.bim_file)
    res_value = f"{args.resolution:.2f}m"
    res_value = res_value.replace(".", "_")  # replace '.' with '_' for file naming
    file_name = f"{base_name}_res_{res_value}_floorplan.png"
    print(f"Generating floorplan from {args.bim_file} and saving to {file_name}")
    data = parse_bim_file(args.bim_file)

    os.chdir(
        args.output
    ) if args.output else None  # Change to output directory if specified
    wall_lines = compute_wall_lines(data)

    floorplan_image, wall_lines_positions = generate_floorplan_image(
        wall_lines, 1 / args.resolution
    )
    write_walls_file(wall_lines_positions, base_name)  # Save the wall lines to a file
    cv2.imwrite(file_name, floorplan_image)  # Save the generated floorplan image
    colorized_file_name = f"{base_name}_res_{res_value}_colorized_floorplan.png"

    flooded_image, colorized_image = flood_pixels(
        floorplan_image.copy(), (255, 0, 0)
    )  # Flood fill with red color
    cv2.imwrite(
        colorized_file_name, colorized_image
    )  # Save the colorized floorplan image

    write_permissive_space_from_flooded_image(
        flooded_image, f"{base_name}_res_{res_value}_floorplan"
    )
