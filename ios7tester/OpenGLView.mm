//
//  OpenGLView.m
//  ColorSpacious
//
//  Created by Sam Fox on 6/16/13.
//  Copyright (c) 2013 Sam Fox. All rights reserved.
//

#import "OpenGLView.h"




@implementation OpenGLView

CC3Vector rotation;
float scalefactor = 600;
int lastPixelCount = 0;
int arraysize = 7008;
//float cube[] = {
//    1.0f,1.0f,1.0f,
//    1.0f,1.0f,0.0f,
//    1.0f,0.0f,1.0f,
//    0.0f,1.0f,1.0f,
//    0.0f,0.0f,1.0f,
//    1.0f,0.0f,0.0f,
//    0.0f,1.0f,0.0f,
//    0.0f,0.0f,0.0f
//};
float xRot = 0;
float yRot = 0;
float zRot = 0;
const int cube[18][3] = {
    // Front
    {-1, -1, -1},
    { -1, -1, 1},
    {-1, 1, 1},
    {-1, 1, -1},
    // Back
    {-1, -1, -1},
    {1, -1, -1},
    {1, -1, 1},
    {1, 1, 1},
    // Left
    {1, 1, -1},
    {1, -1, -1},
    {1, 1, -1},
    {-1, 1, -1},     // Right
    
    {1, 1, -1},
    {1, 1, 1},
    {-1, 1, 1},
    {1, 1, 1},
    // Top
    {1, -1, 1},
    {-1, -1, 1}
  };
float zoom = 1;
float zoomCoef = 1;
- (void)setupVBOs {
    
    GLuint vertexBuffer;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_DYNAMIC_DRAW);
    
    GLuint indexBuffer;
    glGenBuffers(1, &indexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices), Indices, GL_DYNAMIC_DRAW);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

- (void)updateBuffers{
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices), Indices, GL_DYNAMIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_DYNAMIC_DRAW);

}

- (void)setupDisplayLink {
    CADisplayLink* displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(render:)];
    [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
   
    //GPUImageFilter *customfilter = [[GPUImageFilter alloc]init];
}



- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
        [self setupLayer];
        [self setupContext];
        [self setupRenderBuffer];
        [self setupFrameBuffer];
        [self compileShaders];
        [self updateVertices];
        [self setupVBOs];
        [self setupDisplayLink];
        rotation = CC3VectorMake(0, 0, 0);
        UIPinchGestureRecognizer *zoomPinch = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handleZoom:)];
        [self addGestureRecognizer:zoomPinch];
        
           }
    return self;
}

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event{
    UITouch *touch = [[event allTouches] anyObject];
    self.touchDown = [touch locationInView:self];
    yRot= 0;
    xRot = 0;
    zRot = 0;
}

-(void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event{
    UITouch *touch = [[event allTouches] anyObject];
    if(sqrt(powf(fabs(self.touchDown.y - [touch locationInView:self].y),2) + powf(fabs(self.touchDown.x - [touch locationInView:self].x),2)) < 50){
    if (fmod(xRot, M_PI) > M_PI / 2 ) {
        zRot = self.touchDown.y - [touch locationInView:self].y;
    }
    else{
        xRot = self.touchDown.y - [touch locationInView:self].y;
    }
    
    yRot = self.touchDown.x - [touch locationInView:self].x;
    
    
    }

    self.touchDown = [touch locationInView:self];
   }


-(void)handleZoom:(UIPinchGestureRecognizer*)sender{
    zoomCoef = sender.scale;
    if (sender.state == UIGestureRecognizerStateEnded) {
        zoom *= zoomCoef;
        zoomCoef = 1;
    }
}



typedef struct {
    float Position[3];
    float Color[4];
} Vertex;

Vertex Vertices[10000];

GLubyte Indices[10000];


+ (Class)layerClass{
    return [CAEAGLLayer class];
}

-(void)updateVertices{
    
    int size = arraysize - 8;
    for (int i = 0; i < 10000; i ++) {
        Vertices[i].Position[0] = 0;//(i%100) / 100.0f;
        Vertices[i].Position[1] = 0;//(i/100)/100.0f;
        Vertices[i].Position[2] = 0;//self.camera.colors[i*3 + 2]/255.0;
        Vertices[i].Color[0] = 0.0f;//self.camera.colors[i*4] /255.0;
        Vertices[i].Color[1] = 0.0f;//self.camera.colors[i*4 + 1]/255.0;
        Vertices[i].Color[2] = 0.0f;//self.camera.colors[i*4 + 2]/255.0;
        Vertices[i].Color[3] = 0.0f;
        Indices[i] = i;
    }
    for (int i =0; i < 18; i++ ){
        Vertices[arraysize - i].Position[0] = cube[i][0];//(i%100) / 100.0f;
        Vertices[arraysize - i].Position[1] = cube[i][1];//(i/100)/100.0f;
        Vertices[arraysize - i].Position[2] = cube[i][2];//self.camera.colors[i*3 + 2]/255.0;
        Vertices[arraysize - i].Color[0] = 0.0f;//self.camera.colors[i*4] /255.0;
        Vertices[arraysize - i].Color[1] = 0.0f;//self.camera.colors[i*4 + 1]/255.0;
        Vertices[arraysize - i].Color[2] = 0.0f;//self.camera.colors[i*4 + 2]/255.0;
        Vertices[arraysize - i].Color[3] = 0.5f;
        Indices[arraysize - i] = i;
    }

}





-(void)setupLayer {
    _eaglLayer = (CAEAGLLayer*) self.layer;
    _eaglLayer.opaque = YES;
}

-(void)setupContext {
    EAGLRenderingAPI api = kEAGLRenderingAPIOpenGLES2;
    _context =[[EAGLContext alloc] initWithAPI:api];
    if (!_context){
        NSLog(@"Failed to initialize OpenGLES 2.0 context");
        exit(1);
    }
    if (![EAGLContext setCurrentContext:_context]) {
        NSLog(@"Failed to set current OpenGL context");
        exit(1);
    }
}

- (void) setupRenderBuffer{
    glGenRenderbuffers(1, &_colorRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _colorRenderBuffer);
    [_context renderbufferStorage:GL_RENDERBUFFER fromDrawable:_eaglLayer];
}

- (void) setupFrameBuffer{
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER,framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _colorRenderBuffer);
}

- (void)render:(CADisplayLink*)displayLink{
    @autoreleasepool {
        
    
    glClearColor(1, 1, 1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    [self updateBuffers];
    // 1
    CC3GLMatrix *projection = [CC3GLMatrix matrix];
        float num = 2.0f;
    float h = num * self.frame.size.height / self.frame.size.width;
    [projection populateFromFrustumLeft:-1*num andRight:num andBottom:-h/2 andTop:h/2 andNear:num andFar:30];

    glUniformMatrix4fv(_projectionUniform, 1, 0, projection.glMatrix);
    
    CC3GLMatrix *modelView = [CC3GLMatrix matrix];
    [modelView populateFromTranslation:CC3VectorMake(0, 0, -5*zoom*zoomCoef)];
    _currentRotation += displayLink.duration * 90;
    xRot *= 0.95;
    yRot *= 0.95;
    zRot *= 0.95;
    
    rotation =CC3VectorMake(rotation.x - 0.5 * xRot, rotation.y - 0.5 * yRot, rotation.z - 0.5 * zRot);
    [modelView rotateBy:rotation];
    glUniformMatrix4fv(_modelViewUniform, 1, 0, modelView.glMatrix);
    glViewport(0, 0, self.frame.size.width, self.frame.size.height);
    
    // 2
    glVertexAttribPointer(_positionSlot, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer(_colorSlot, 4, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) * 3));
       
    cv::vector<float> pixels = [self getPixelsWithHorizDensity:5 andVertDensity:3];
    
        
        float max_mag = 0;
        std::cout << pixels.size() << "," << self.colorsToRender.size() << std::endl;
        for(int k = 0; k < fmax(pixels.size(), lastPixelCount); k += 3){
            if (k >= pixels.size() - 1) {
                Vertices[k/3].Position[0] =0;
                Vertices[k/3].Position[1] = 0;
                Vertices[k/3].Position[2] = 0;
                Vertices[k/3].Color[3] = 0.0f;
            }
            else{
                Vertices[k/3].Position[0] =( scalefactor * (pixels.at(k) / 256.0f) );
                Vertices[k/3].Position[1] = ( scalefactor * (pixels.at(k + 1) / 256.0f) );
                Vertices[k/3].Position[2] = ( scalefactor * (pixels.at(k + 2) / 256.0f) );
                Vertices[k/3].Color[3] = 1.0f;
                Vertices[k/3].Color[0] =  0.0f;
                Vertices[k/3].Color[1] =  0.0f;
                Vertices[k/3].Color[2] =  0.0f;
                //if(k/3 < self.colorsToRender.size()){
                    //Vertices[k/3].Color[0] =  self.colorsToRender.at(k/3)[0] / 255.0f;
                    //Vertices[k/3].Color[1] =  self.colorsToRender.at(k/3)[1] / 255.0f;
                    //Vertices[k/3].Color[2] =  self.colorsToRender.at(k/3)[2] / 255.0f;
                //}
            }
            
            
        }
        
        lastPixelCount = pixels.size();

        

    glDrawArrays(GL_POINTS, 0, arraysize-20);
    glDrawArrays(GL_LINES, arraysize - 18, 18);
       
    [_context presentRenderbuffer:GL_RENDERBUFFER];
    }
}

- (cv::vector<float>)getPixelsWithHorizDensity:(int)xres andVertDensity:(int)yres{
    
    self.dontTouch = TRUE;
    cv::vector<float> output;
    if (self.pixelsToRender.size() > 2) {
        float maxMag = 0;
        for (int k = 0; k < self.pixelsToRender.size(); k ++) {
                        cv::Point3d point = self.pixelsToRender[k];
            float mag = sqrt(point.dot(point));
            if (mag > maxMag) {
                maxMag = mag;
            }
        }
        
        for (int i = 0; i < self.pixelsToRender.size(); i ++) {
            
            cv::Point3d point = self.pixelsToRender.at(i);
            output.push_back(point.x / maxMag);
            output.push_back(point.y / maxMag);
            output.push_back(point.z / maxMag);

        }
    }
    self.dontTouch = false;
    return output;
}

- (GLuint)compileShader:(NSString*)shaderName withType:(GLenum)shaderType {
    
    // 1
    NSString* shaderPath = [[NSBundle mainBundle] pathForResource:shaderName
                                                           ofType:@"glsl"];
    NSError* error;
    NSString* shaderString = [NSString stringWithContentsOfFile:shaderPath
                                                       encoding:NSUTF8StringEncoding error:&error];
    if (!shaderString) {
        NSLog(@"Error loading shader: %@", error.localizedDescription);
        exit(1);
    }
    
    // 2
    GLuint shaderHandle = glCreateShader(shaderType);
    
    // 3
    const char * shaderStringUTF8 = [shaderString UTF8String];
    int shaderStringLength = [shaderString length];
    glShaderSource(shaderHandle, 1, &shaderStringUTF8, &shaderStringLength);
    
    // 4
    glCompileShader(shaderHandle);
    
    // 5
    GLint compileSuccess;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compileSuccess);
    if (compileSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetShaderInfoLog(shaderHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    return shaderHandle;
    
}

- (void)compileShaders {
    
    // 1
    GLuint vertexShader = [self compileShader:@"Vertex"
                                     withType:GL_VERTEX_SHADER];
    GLuint fragmentShader = [self compileShader:@"Fragment"
                                       withType:GL_FRAGMENT_SHADER];
    
    // 2
    GLuint programHandle = glCreateProgram();
    glAttachShader(programHandle, vertexShader);
    glAttachShader(programHandle, fragmentShader);
    glLinkProgram(programHandle);
    
    // 3
    GLint linkSuccess;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
    if (linkSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetProgramInfoLog(programHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    // 4
    glUseProgram(programHandle);
    
    // 5
    _positionSlot = glGetAttribLocation(programHandle, "Position");
    _colorSlot = glGetAttribLocation(programHandle, "SourceColor");
    glEnableVertexAttribArray(_positionSlot);
    glEnableVertexAttribArray(_colorSlot);
    _projectionUniform = glGetUniformLocation(programHandle, "Projection");
    _modelViewUniform = glGetUniformLocation(programHandle, "Modelview");

}

@end
