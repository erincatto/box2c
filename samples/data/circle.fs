// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

in vec2 f_position;
in vec4 f_color;
in float f_zoom;

out vec4 color;

void main()
{
    // radius in unit quad
    float radius = 1.0;

    // distance to circle
    vec2 w = f_position;
    float dw = sqrt(dot(w, w));
    float d = abs(dw - radius);

    // scale border by zoom so the pixel width is constant
    float borderThickness = 0.07f * f_zoom;

    color = vec4(f_color.rgb, 1.0 - smoothstep(0.0, borderThickness, d));
}
