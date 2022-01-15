#include "glue.h"


Color GetColor2(int hexValue)
{
    Color color;

    color.r = (unsigned char)(hexValue >> 0) & 0xFF ;
    color.g = (unsigned char)(hexValue >> 8) & 0xFF ;
    color.b = (unsigned char)(hexValue >> 16) & 0xFF ;
    color.a = (unsigned char)(hexValue >> 24) & 0xFF ;

    return color;
}


ImVec2 operator*(const float f, const ImVec2& v)
{
	return ImVec2{f * v.x, f * v.y};
}

ImVec2 operator+(const ImVec2& a, const ImVec2& b)
{
	return ImVec2{a.x + b.x, a.y + b.y};
}

ImVec2 operator-(const ImVec2& a, const ImVec2& b)
{
	return ImVec2{a.x - b.x, a.y - b.y};
}

bool operator!=(const ImVec2& a, const ImVec2& b)
{
	return a.x != b.x || a.y != b.y;
}

ImVec4 operator*(const float f, const ImVec4& v)
{
	return ImVec4{f * v.x, f * v.y, f * v.z, f * v.w};
}

ImVec4 operator+(const ImVec4& a, const ImVec4& b)
{
	return ImVec4{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
float min3(float a, float b, float c)
{
	if (a < b && a < c) { return a; }
	return b < c ? b : c;
}

float max3(float a, float b, float c)
{
	if (a > b && a > c) { return a; }
	return b > c ? b : c;
}




	struct Rect
	{
		float MinX, MinY, MaxX, MaxY;
		float MinU, MinV, MaxU, MaxV;

		bool IsOnExtreme(const ImVec2& point) const
		{
			return (point.x == MinX || point.x == MaxX) && (point.y == MinY || point.y == MaxY);
		}

		bool UsesOnlyColor() const
		{
			const ImVec2& whitePixel = ImGui::GetIO().Fonts->TexUvWhitePixel;
			return MinU == MaxU && MinU == whitePixel.x && MinV == MaxV && MaxV == whitePixel.y;
		}


		static Rect CalculateBoundingBox(const ImDrawVert& v0, const ImDrawVert& v1, const ImDrawVert& v2)
		{
			 Rect r;

			 r.MinX =min3(v0.pos.x, v1.pos.x, v2.pos.x );
			 r.MinY =min3(v0.pos.y, v1.pos.y, v2.pos.y );
			 r.MaxX =max3(v0.pos.x, v1.pos.x, v2.pos.x );
			 r.MaxY =max3(v0.pos.y, v1.pos.y, v2.pos.y );

			 r.MinU =min3( v0.uv.x, v1.uv.x, v2.uv.x);
			 r.MinV =min3( v0.uv.y, v1.uv.y, v2.uv.y);
			 r.MaxU=max3 ( v0.uv.x, v1.uv.x, v2.uv.x);
			 r.MaxV=max3 ( v0.uv.y, v1.uv.y, v2.uv.y);
			 return r;
		}
	};





static double g_Time = 0.0;
Texture2D*  g_FontTexture;
Texture2D*   testeimage;
Image image ;



static const char* ImGui_ImplRaylib_GetClipboardText(void*)
{
    return GetClipboardText();
}

static void ImGui_ImplRaylib_SetClipboardText(void*, const char* text)
{
    SetClipboardText(text);
}

bool ImGui_ImplRaylib_CreateFontsTexture()
{
    // Build texture atlas
    ImGuiIO& io = ImGui::GetIO();
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   // Load as RGBA 32-bit (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
    g_FontTexture = new Texture();
    g_FontTexture->width = width;
    g_FontTexture->height = height;
    g_FontTexture->mipmaps = 1;
    g_FontTexture->format = UNCOMPRESSED_R8G8B8A8;
    g_FontTexture->id= rlLoadTexture(pixels,width,height,UNCOMPRESSED_R8G8B8A8,1);
    io.Fonts->TexID = (void*)g_FontTexture;

    return true;
}

bool ImGui_ImplRaylib_Init()
{

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
   // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
   // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;       // We can honor GetMouseCursor() values (optional)
    io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;        // We can honor io.WantSetMousePos requests (optional, rarely used)




    io.BackendPlatformName = "imgui_impl_raylib";

    io.KeyMap[ImGuiKey_Tab] = KEY_TAB;
    io.KeyMap[ImGuiKey_LeftArrow] = KEY_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = KEY_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = KEY_UP;
    io.KeyMap[ImGuiKey_DownArrow] = KEY_DOWN;
    io.KeyMap[ImGuiKey_PageUp] = KEY_PAGE_DOWN;
    io.KeyMap[ImGuiKey_PageDown] = KEY_PAGE_UP;
    io.KeyMap[ImGuiKey_Home] = KEY_HOME;
    io.KeyMap[ImGuiKey_End] = KEY_END;
    io.KeyMap[ImGuiKey_Insert] = KEY_INSERT;
    io.KeyMap[ImGuiKey_Delete] = KEY_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = KEY_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = KEY_SPACE;
    io.KeyMap[ImGuiKey_Enter] = KEY_ENTER;
    io.KeyMap[ImGuiKey_Escape] = KEY_ESCAPE;
    io.KeyMap[ImGuiKey_KeyPadEnter] = KEY_KP_ENTER;
    io.KeyMap[ImGuiKey_A] = KEY_A;
    io.KeyMap[ImGuiKey_C] = KEY_C;
    io.KeyMap[ImGuiKey_V] = KEY_V;
    io.KeyMap[ImGuiKey_X] = KEY_X;
    io.KeyMap[ImGuiKey_Y] = KEY_Y;
    io.KeyMap[ImGuiKey_Z] = KEY_Z;

    io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);

    io.SetClipboardTextFn = ImGui_ImplRaylib_SetClipboardText;
    io.GetClipboardTextFn = ImGui_ImplRaylib_GetClipboardText;
    io.ClipboardUserData = NULL;

    ImGui_ImplRaylib_CreateFontsTexture();

    return true;
}

void ImGui_ImplRaylib_Shutdown()
{
    g_Time = 0.0;
}

static void ImGui_ImplRaylib_UpdateMousePosAndButtons()
{
    ImGuiIO& io = ImGui::GetIO();

    // Set OS mouse position if requested (rarely used, only when ImGuiConfigFlags_NavEnableSetMousePos is enabled by user)
    if (io.WantSetMousePos)
        SetMousePosition(io.MousePos.x, io.MousePos.y);
    else
        io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);

    io.MouseDown[0] = IsMouseButtonDown(MOUSE_LEFT_BUTTON);
    io.MouseDown[1] = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);
    io.MouseDown[2] = IsMouseButtonDown(MOUSE_MIDDLE_BUTTON);

    if (!IsWindowMinimized())
        io.MousePos = ImVec2(GetMouseX(), GetMouseY());
}

static void ImGui_ImplRaylib_UpdateMouseCursor()
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange)
        return;

    ImGuiMouseCursor imgui_cursor = ImGui::GetMouseCursor();
    if (io.MouseDrawCursor || imgui_cursor == ImGuiMouseCursor_None)
    {
        // Hide OS mouse cursor if imgui is drawing it or if it wants no cursor
        HideCursor();
    }
    else
    {
        // Show OS mouse cursor
        ShowCursor();
    }
}

void ImGui_ImplRaylib_NewFrame()
{
    ImGuiIO &io = ImGui::GetIO();
    IM_ASSERT(io.Fonts->IsBuilt() && "Font atlas not built! It is generally built by the renderer backend. Missing call to renderer _NewFrame() function? e.g. ImGui_ray_NewFrame().");


    io.DisplaySize = ImVec2((float) GetScreenWidth(), (float) GetScreenHeight());

    double current_time = GetTime();
    io.DeltaTime = g_Time > 0.0 ? (float)(current_time - g_Time) : (float)(1.0f/60.0f);
    g_Time = current_time;

    io.KeyCtrl = IsKeyDown(KEY_RIGHT_CONTROL) || IsKeyDown(KEY_LEFT_CONTROL);
    io.KeyShift = IsKeyDown(KEY_RIGHT_SHIFT) || IsKeyDown(KEY_LEFT_SHIFT);
    io.KeyAlt = IsKeyDown(KEY_RIGHT_ALT) || IsKeyDown(KEY_LEFT_ALT);
    io.KeySuper = IsKeyDown(KEY_RIGHT_SUPER) || IsKeyDown(KEY_LEFT_SUPER);

    ImGui_ImplRaylib_UpdateMousePosAndButtons();
    ImGui_ImplRaylib_UpdateMouseCursor();

    if (GetMouseWheelMove() > 0)
        io.MouseWheel += 1;
    else if (GetMouseWheelMove() < 0)
        io.MouseWheel -= 1;

            ImGui::NewFrame();
}

#define FOR_ALL_KEYS(X) \
    X(KEY_APOSTROPHE); \
    X(KEY_COMMA); \
    X(KEY_MINUS); \
    X(KEY_PERIOD); \
    X(KEY_SLASH); \
    X(KEY_ZERO); \
    X(KEY_ONE); \
    X(KEY_TWO); \
    X(KEY_THREE); \
    X(KEY_FOUR); \
    X(KEY_FIVE); \
    X(KEY_SIX); \
    X(KEY_SEVEN); \
    X(KEY_EIGHT); \
    X(KEY_NINE); \
    X(KEY_SEMICOLON); \
    X(KEY_EQUAL); \
    X(KEY_A); \
    X(KEY_B); \
    X(KEY_C); \
    X(KEY_D); \
    X(KEY_E); \
    X(KEY_F); \
    X(KEY_G); \
    X(KEY_H); \
    X(KEY_I); \
    X(KEY_J); \
    X(KEY_K); \
    X(KEY_L); \
    X(KEY_M); \
    X(KEY_N); \
    X(KEY_O); \
    X(KEY_P); \
    X(KEY_Q); \
    X(KEY_R); \
    X(KEY_S); \
    X(KEY_T); \
    X(KEY_U); \
    X(KEY_V); \
    X(KEY_W); \
    X(KEY_X); \
    X(KEY_Y); \
    X(KEY_Z); \
    X(KEY_SPACE); \
    X(KEY_ESCAPE); \
    X(KEY_ENTER); \
    X(KEY_TAB); \
    X(KEY_BACKSPACE); \
    X(KEY_INSERT); \
    X(KEY_DELETE); \
    X(KEY_RIGHT); \
    X(KEY_LEFT); \
    X(KEY_DOWN); \
    X(KEY_UP); \
    X(KEY_PAGE_UP); \
    X(KEY_PAGE_DOWN); \
    X(KEY_HOME); \
    X(KEY_END); \
    X(KEY_CAPS_LOCK); \
    X(KEY_SCROLL_LOCK); \
    X(KEY_NUM_LOCK); \
    X(KEY_PRINT_SCREEN); \
    X(KEY_PAUSE); \
    X(KEY_F1); \
    X(KEY_F2); \
    X(KEY_F3); \
    X(KEY_F4); \
    X(KEY_F5); \
    X(KEY_F6); \
    X(KEY_F7); \
    X(KEY_F8); \
    X(KEY_F9); \
    X(KEY_F10); \
    X(KEY_F11); \
    X(KEY_F12); \
    X(KEY_LEFT_SHIFT); \
    X(KEY_LEFT_CONTROL); \
    X(KEY_LEFT_ALT); \
    X(KEY_LEFT_SUPER); \
    X(KEY_RIGHT_SHIFT); \
    X(KEY_RIGHT_CONTROL); \
    X(KEY_RIGHT_ALT); \
    X(KEY_RIGHT_SUPER); \
    X(KEY_KB_MENU); \
    X(KEY_LEFT_BRACKET); \
    X(KEY_BACKSLASH); \
    X(KEY_RIGHT_BRACKET); \
    X(KEY_GRAVE); \
    X(KEY_KP_0); \
    X(KEY_KP_1); \
    X(KEY_KP_2); \
    X(KEY_KP_3); \
    X(KEY_KP_4); \
    X(KEY_KP_5); \
    X(KEY_KP_6); \
    X(KEY_KP_7); \
    X(KEY_KP_8); \
    X(KEY_KP_9); \
    X(KEY_KP_DECIMAL); \
    X(KEY_KP_DIVIDE); \
    X(KEY_KP_MULTIPLY); \
    X(KEY_KP_SUBTRACT); \
    X(KEY_KP_ADD); \
    X(KEY_KP_ENTER); \
    X(KEY_KP_EQUAL);

#define SET_KEY_DOWN(KEY) io.KeysDown[KEY] = IsKeyDown(KEY)

bool ImGui_ImplRaylib_ProcessEvent()
{
    ImGuiIO& io = ImGui::GetIO();

    FOR_ALL_KEYS(SET_KEY_DOWN);

    // Uncomment the three lines below if using raylib earlier than version 3.
    //if (GetKeyPressed() != -1)
    //{
        io.AddInputCharacter(GetKeyPressed());
    //}

    return true;
}



void DrawUniformColorTriangle(const ImDrawVert& v0, const ImDrawVert& v1, const ImDrawVert& v2)
{



rlBegin(RL_TRIANGLES);




                    Color colora= GetColor2(v0.col);
                    Color colorb= GetColor2(v1.col);
                    Color colorc= GetColor2(v2.col);


                   rlVertex2f(v0.pos.x,v0.pos.y);
             //      rlTexCoord2f(v0.uv.x,v0.uv.y);
                   rlColor4ub(colora.r,colora.g,colora.b,colora.a);

                   rlVertex2f(v1.pos.x,v1.pos.y);
              //     rlTexCoord2f(v1.uv.x,v1.uv.y);
                   rlColor4ub(colorb.r,colorb.g,colorb.b,colorb.a);

                   rlVertex2f(v2.pos.x,v2.pos.y);
               //    rlTexCoord2f(v2.uv.x,v2.uv.y);
                   rlColor4ub(colorc.r,colorc.g,colorc.b,colorc.a);




//


               rlEnd();



}



void render_primitive(const Rect& bounding, Texture* texture, const Color& color, bool doHorizontalFlip, bool doVerticalFlip)
{

int textureWidth=texture->width;
int textureHeight=texture->height;

Rectangle destination = {
			(bounding.MinX),
			(bounding.MinY),
			(bounding.MaxX - bounding.MinX),
			(bounding.MaxY - bounding.MinY)
		};


Rectangle source = {
				(bounding.MinU * textureWidth),
				(bounding.MinV * textureHeight),
				((bounding.MaxU - bounding.MinU) * textureWidth),
				((bounding.MaxV - bounding.MinV) * textureHeight)



};

	if (bounding.UsesOnlyColor())
		{


             DrawTexturePro(*texture,source,destination,{0,0},0,color);
		}
		else
		{


        DrawTexturePro(*texture,source,destination,{0,0},0,color);
}

}


void BeginScissor(int x, int y, int width, int height)
{
    rlglDraw(); // Force drawing elements

    rlEnableScissorTest();
   // rlScissor(x, GetScreenHeight() - (y + height), width, height);
    rlScissor(x, y, width, height);
}

// End scissor mode
void EndScissor(void)
{
    rlglDraw(); // Force drawing elements
    rlDisableScissorTest();
}

void ImGui_ImplRay_RenderDrawData(ImDrawData* drawData)
{

rlDisableDepthTest();
rlDisableBackfaceCulling();


    const float width_points = ImGui::GetIO().DisplaySize.x;
	const float height_points = ImGui::GetIO().DisplaySize.y;
//	const ImVec2 scale{width_pixels / width_points, height_pixels / height_points};
	//PaintTarget target{pixels, width_pixels, height_pixels, scale};
//	const ImDrawData* draw_data = ImGui::GetDrawData();


    int fb_width = (int)(drawData->DisplaySize.x * drawData->FramebufferScale.x);
    int fb_height = (int)(drawData->DisplaySize.y * drawData->FramebufferScale.y);
    if (fb_width == 0 || fb_height == 0)
        return;


    ImVec2 clip_off = drawData->DisplayPos;         // (0,0) unless using multi-viewports
    ImVec2 clip_scale = drawData->FramebufferScale; // (1,1) unless using retina display which are often (2,2)



		ImGuiIO& io = ImGui::GetIO();

		for (int n = 0; n < drawData->CmdListsCount; n++)
		{
			auto commandList = drawData->CmdLists[n];
			auto vertexBuffer = commandList->VtxBuffer;
			auto indexBuffer = commandList->IdxBuffer.Data;

			for (int cmd_i = 0; cmd_i < commandList->CmdBuffer.Size; cmd_i++)
			{
				const ImDrawCmd* drawCommand = &commandList->CmdBuffer[cmd_i];



				if (drawCommand->UserCallback)
				{
					drawCommand->UserCallback(commandList, drawCommand);
				}
				else
				{
					const bool isWrappedTexture = drawCommand->TextureId == io.Fonts->TexID;
					rlEnableTexture((unsigned int)(intptr_t)drawCommand->TextureId);
					//rlEnableTexture((unsigned int)(Texture2D *)drawCommand->TextureId);

				//	SetMaterialTexture(&mat, MAP_DIFFUSE, *(Texture2D *)pcmd->TextureId);
///media/djoker/code/linux/basic/raymonkey/modules/imgui/native/rayteste/src/glue.cpp|523|error: ‘intptr_t’ was not declared in this scope|

  			   ImVec4 clip_rect;
                clip_rect.x = (drawCommand->ClipRect.x - clip_off.x) * clip_scale.x;
                clip_rect.y = (drawCommand->ClipRect.y - clip_off.y) * clip_scale.y;
                clip_rect.z = (drawCommand->ClipRect.z - clip_off.x) * clip_scale.x;
                clip_rect.w = (drawCommand->ClipRect.w - clip_off.y) * clip_scale.y;


    		     if (clip_rect.x < fb_width && clip_rect.y < fb_height && clip_rect.z >= 0.0f && clip_rect.w >= 0.0f)
                {

                BeginScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w), (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));
             //   BeginScissorMode((int)clip_rect.x, (int)(clip_rect.y), (int)(clip_rect.z), (int)( clip_rect.w));


					// Loops over triangles.
					for (unsigned int i = 0; i + 3 <= drawCommand->ElemCount; i += 3)
					{
						const ImDrawVert& v0 = vertexBuffer[indexBuffer[i + 0]];
						const ImDrawVert& v1 = vertexBuffer[indexBuffer[i + 1]];
						const ImDrawVert& v2 = vertexBuffer[indexBuffer[i + 2]];

						const Rect& bounding = Rect::CalculateBoundingBox(v0, v1, v2);

						const bool isTriangleUniformColor = v0.col == v1.col && v1.col == v2.col;
						const bool doesTriangleUseOnlyColor = bounding.UsesOnlyColor();

						// Actually, since we render a whole bunch of rectangles, we try to first detect those, and render them more efficiently.
						// How are rectangles detected? It's actually pretty simple: If all 6 vertices lie on the extremes of the bounding box,
						// it's a rectangle.
						if (i + 6 <= drawCommand->ElemCount)
						{
							const ImDrawVert& v3 = vertexBuffer[indexBuffer[i + 3]];
							const ImDrawVert& v4 = vertexBuffer[indexBuffer[i + 4]];
							const ImDrawVert& v5 = vertexBuffer[indexBuffer[i + 5]];

							const bool isUniformColor = isTriangleUniformColor && v2.col == v3.col && v3.col == v4.col && v4.col == v5.col;
	                     if (isUniformColor
							&& bounding.IsOnExtreme(v0.pos)
							&& bounding.IsOnExtreme(v1.pos)
							&& bounding.IsOnExtreme(v2.pos)
							&& bounding.IsOnExtreme(v3.pos)
							&& bounding.IsOnExtreme(v4.pos)
							&& bounding.IsOnExtreme(v5.pos)
							)
							{
								// ImGui gives the triangles in a nice order: the first vertex happens to be the topleft corner of our rectangle.
								// We need to check for the orientation of the texture, as I believe in theory ImGui could feed us a flipped texture,
								// so that the larger texture coordinates are at topleft instead of bottomright.
								// We don't consider equal texture coordinates to require a flip, as then the rectangle is mostlikely simply a colored rectangle.
								const bool doHorizontalFlip = v2.uv.x < v0.uv.x;
								const bool doVerticalFlip = v2.uv.x < v0.uv.x;





								//printf(" %i %i \n",doHorizontalFlip,doVerticalFlip);
                                  Color color= GetColor2(v0.col);
								if (isWrappedTexture)
								{


									render_primitive(bounding, static_cast< Texture*>(drawCommand->TextureId),color, false,false);
//									drawTriangle(v0, v1, v2,v3, v4, v5);

								}
								else
								{
								//printf(" no texture \n");
									render_primitive(bounding, static_cast< Texture*>(drawCommand->TextureId),color, doHorizontalFlip, doVerticalFlip);
								//	DrawRectangle(bounding, static_cast<SDL_Texture*>(drawCommand->TextureId), Color(v0.col), doHorizontalFlip, doVerticalFlip);
								}

								i += 3;  // Additional increment to account for the extra 3 vertices we consumed.
								continue;
							}
						}


						if (isTriangleUniformColor)
						{
							DrawUniformColorTriangle(v0, v1, v2);
							}
						else
						{

							DrawUniformColorTriangle(v0, v1, v2);
						}
					}//lop
					EndScissor();
                  } //clip
				}// else

				indexBuffer += drawCommand->ElemCount;
			}
		}






}



	// After raylib initialization
void InitializeImGui()
{
ImGui_ImplRaylib_Init();

}

void BeginImGui()
{
		ImGui_ImplRaylib_NewFrame();
        ImGui_ImplRaylib_ProcessEvent();
}
void EndImGui()
{
ImGui::Render();
ImGui_ImplRay_RenderDrawData(ImGui::GetDrawData());

}
void FinalizeImGui()
{
   ImGui_ImplRaylib_Shutdown()  ;
   ImGui::DestroyContext();

}

