import argparse
import os
from glob import glob

parser = argparse.ArgumentParser(description="Convert .dae files to the .obj format.")
parser.add_argument(
    "-i",
    "--input",
    required=True,
    type=str,
    help="Input directory containing .dae files.",
)
parser.add_argument(
    "-o",
    "--output",
    required=True,
    type=str,
    help="Output directory to save .obj files.",
)
args = parser.parse_args()

os.makedirs(args.output, exist_ok=True)
for dae_file in glob(os.path.join(args.input, "*.dae")):
    print(f"Converting {dae_file}...")
    out_file = os.path.join(args.output, os.path.basename(dae_file).replace(".dae", ".obj"))
    os.system(f"meshlabserver -i {dae_file} -o {out_file} -m vn")
    print(f"Saved to {out_file}.")
