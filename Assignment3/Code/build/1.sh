ss=$1
./Rasterizer "output${ss}_texture.png" texture
./Rasterizer "output${ss}_normal.png" normal
./Rasterizer "output${ss}_phong.png" phong
./Rasterizer "output${ss}_bump.png" bump
./Rasterizer "output${ss}_disaplacement.png" displacement
