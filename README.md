# DOE---PCB-Detection

•	Lead development of an automated PCB manufacturing defect detection system for the Department of Energy using a multi-modal imaging pipeline (RGB, UV, IR) at varying heights and angles, implementing Gaussian splatting for 3D reconstruction. Trained ML-based solder defect classification model
•	Wrote STM32 embedded firmware (C/HAL) to control LED illumination pods; designed PCBs for LED pods and integrated ZWO astronomy camera and WeMacro rail actuator via custom Python drivers
•	Built full UX and backend (Python) for the inspection system, integrating camera control, rail positioning, autofocus capabilities and live image acquisition into a single interface
•	Multi-modal imaging system capturing RGB, UV, and IR images at variable heights and angles to build a 3D surface model of PCBs; Gaussian splatting compares scans against ground-truth board state to flag spatial deviations
•	Trained ML classification model to identify specific defect categories (solder bridges, cold joints, tombstoning, missing components) from reconstructed point-cloud and image features
•	Implemented STM32 embedded firmware (C/HAL) for LED pod control; designed hardware drivers for ZWO camera (Python/ASCOM) and WeMacro motorized rail (Python/serial) with full UX in Python/Tkinter
Created full CAD file of entire lightproof box model, desing, iteration, prototype
