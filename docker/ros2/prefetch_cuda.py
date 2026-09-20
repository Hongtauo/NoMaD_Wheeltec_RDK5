"""Optional multi-connection wheel download from official indexes with SHA256.

Requires aria2c. Downloads only; installation remains a separate pip operation.
"""
import argparse
import json
from pathlib import Path
import subprocess
from html.parser import HTMLParser
from urllib.parse import unquote, urljoin, urlsplit
from urllib.request import urlopen

from pip._vendor.packaging.tags import sys_tags
from pip._vendor.packaging.utils import parse_wheel_filename
from pip._vendor.packaging.requirements import Requirement


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        if tag == "a":
            self.links.extend(value for key, value in attrs if key == "href")


def metadata(name, version):
    with urlopen(f"https://pypi.org/pypi/{name}/{version}/json", timeout=60) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", type=Path, default=Path("/tmp/nomad-ros2-wheelhouse"))
    args = parser.parse_args()
    args.destination.mkdir(parents=True, exist_ok=True)
    downloads = []
    for name, version in (("torch", "2.6.0"), ("torchvision", "0.21.0")):
        index = f"https://download.pytorch.org/whl/cu124/{name}/"
        links = Links()
        with urlopen(index, timeout=60) as response:
            links.feed(response.read().decode())
        filename = f"{name}-{version}+cu124-cp310-cp310-linux_x86_64.whl"
        link = next(urljoin(index, value) for value in links.links
                    if unquote(urlsplit(value).path.rsplit("/", 1)[-1]) == filename)
        digest = urlsplit(link).fragment.removeprefix("sha256=")
        assert len(digest) == 64, link
        downloads.append((link.split("#")[0], filename, digest))

    supported = {tag: index for index, tag in enumerate(sys_tags())}
    for declaration in metadata("torch", "2.6.0")["info"]["requires_dist"]:
        requirement = Requirement(declaration)
        if not (requirement.name.startswith("nvidia-") or requirement.name == "triton"):
            continue
        if requirement.marker and not requirement.marker.evaluate():
            continue
        specifier = list(requirement.specifier)
        assert len(specifier) == 1 and specifier[0].operator == "==", declaration
        candidates = []
        for wheel in metadata(requirement.name, specifier[0].version)["urls"]:
            if wheel["packagetype"] != "bdist_wheel":
                continue
            tags = parse_wheel_filename(wheel["filename"])[3]
            ranks = [supported[tag] for tag in tags if tag in supported]
            if ranks:
                candidates.append((min(ranks), wheel))
        _, wheel = min(candidates, key=lambda item: item[0])
        downloads.append((wheel["url"], wheel["filename"], wheel["digests"]["sha256"]))

    lines = []
    for url, filename, digest in downloads:
        assert Path(filename).name == filename
        lines.extend((url, f"  out={filename}", f"  checksum=sha-256={digest}"))
    manifest = args.destination / "aria2-input.txt"
    manifest.write_text("\n".join(lines) + "\n")
    print(f"Downloading {len(downloads)} wheels to {args.destination}", flush=True)
    subprocess.run([
        "aria2c", "--continue=true", "--check-integrity=true", "--auto-file-renaming=false",
        "--allow-overwrite=true", "--file-allocation=none", "--max-connection-per-server=8",
        "--split=8", "--max-concurrent-downloads=4", "--summary-interval=30",
        "--show-console-readout=false",
        "--console-log-level=notice", "--download-result=full", "--enable-color=false",
        "--dir", str(args.destination), "--input-file", str(manifest),
    ], check=True)
    (args.destination / "download-complete.txt").write_text("All wheels passed SHA256 validation.\n")


if __name__ == "__main__":
    main()
