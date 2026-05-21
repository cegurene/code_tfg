from pathlib import Path
from setuptools import find_packages, setup

package_name = 'gui'


def _normalize_generated_ui_imports():
    target_files = [
        (Path(__file__).resolve().parent / package_name / 'gui_assistive_exo_data.py', [
            'try:',
            '    from .mplGraph import FigureCanvasQTAgg',
            'except ImportError:',
            '    from gui.mplGraph import FigureCanvasQTAgg',
        ]),
        (Path(__file__).resolve().parent / package_name / 'gui_impedance' / 'gui_impedance_data.py', [
            'try:',
            '    from ..mplGraph import FigureCanvasQTAgg',
            'except ImportError:',
            '    from gui.mplGraph import FigureCanvasQTAgg',
        ]),
        (Path(__file__).resolve().parent / package_name / 'gui_completa' / 'gui_completa_data.py', [
            'from gui.mplGraph import FigureCanvasQTAgg',
        ]),
    ]

    for generated_ui_py, import_block in target_files:
        if not generated_ui_py.exists():
            continue

        text = generated_ui_py.read_text(encoding='utf-8')
        marker = '\nif __name__ == "__main__":'
        marker_index = text.rfind(marker)
        if marker_index == -1:
            continue

        head = text[:marker_index]
        tail = text[marker_index:]

        head_lines = head.splitlines()
        scan_start = max(0, len(head_lines) - 80)

        cleaned_head_lines = head_lines[:scan_start]
        for line in head_lines[scan_start:]:
            stripped = line.strip()
            if 'FigureCanvasQTAgg' in line:
                continue
            if stripped in {'try:', 'except ImportError:'}:
                continue
            cleaned_head_lines.append(line)

        cleaned_head = '\n'.join(cleaned_head_lines).rstrip()
        updated = f"{cleaned_head}\n" + '\n'.join(import_block) + f"\n{tail.lstrip()}"

        if updated != text:
            generated_ui_py.write_text(updated, encoding='utf-8')


_normalize_generated_ui_imports()

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'gui', 'gui.gui_impedance', 'gui.gui_completa'], #find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='legreplica',
    maintainer_email='marco.iglesias.santos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_assistive_exo = gui.gui_assistive_exo:main',
            'gui_impedance = gui.gui_impedance.gui_impedance:main',
            'gui_completa = gui.gui_completa.gui_completa:main',
        ],
    },
)
