#version 330

in vec2 f_position;
in vec4 f_color;
in float f_zoom;

out vec4 color;

// https://en.wikipedia.org/wiki/Alpha_compositing
vec4 blend_colors(vec4 front, vec4 back)
{
    vec3 cSrc = front.rgb;
    float alphaSrc = front.a;
    vec3 cDst = back.rgb;
    float alphaDst = back.a;

    vec3 cOut = cSrc * alphaSrc + cDst * alphaDst * (1.0 - alphaSrc);
    float alphaOut = alphaSrc + alphaDst * (1.0 - alphaSrc);
    cOut = cOut / alphaOut;

    return vec4(cOut, alphaOut);
}

void main()
{
    // radius in unit quad
    float radius = 1.0;

    // distance to axis line segment
    vec2 e = vec2(radius, 0);
    vec2 w = f_position;
    float we = dot(w, e);
    vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
    float da = sqrt(dot(b, b));

    // distance to circle
    float dw = sqrt(dot(w, w));
    float dc = abs(dw - radius);

    // union of circle and axis
    float d = min(da, dc);

    vec4 borderColor = f_color;
    vec4 fillColor = 0.6f * borderColor;

    // scale border by zoom so the pixel width is constant
    float borderThickness = 0.07f * f_zoom;

    vec4 back = vec4(fillColor.rgb, fillColor.a * (1.0 - smoothstep(radius, radius + borderThickness, dw)));
    vec4 front = vec4(borderColor.rgb, 1.0 - smoothstep(0.0, borderThickness, d));
    color = blend_colors(front, back);
}
