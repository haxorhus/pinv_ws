"""Lógica de teclas y pulsos del teleop, sin ROS ni terminal."""

from pinv_teleop.teleop_teclado import aplicar_tecla, pulsos_estimados, separar_teclas

LIM = {'paso_v': 0.1, 'paso_w': 0.1, 'v_max': 2.0, 'w_max': 1.0}


def test_acelerar_y_flechas():
    assert aplicar_tecla('w', 0.0, 0.0, **LIM) == (0.1, 0.0, False)
    assert aplicar_tecla('\x1b[A', 0.1, 0.0, **LIM) == (0.2, 0.0, False)
    assert aplicar_tecla('\x1b[D', 0.0, 0.0, **LIM) == (0.0, 0.1, False)
    assert aplicar_tecla('\x1b[C', 0.0, 0.0, **LIM) == (0.0, -0.1, False)


def test_sin_reversa_y_limites():
    assert aplicar_tecla('s', 0.0, 0.0, **LIM)[0] == 0.0
    assert aplicar_tecla('w', 2.0, 0.0, **LIM)[0] == 2.0
    assert aplicar_tecla('a', 0.0, 1.0, **LIM)[1] == 1.0
    assert aplicar_tecla('d', 0.0, -1.0, **LIM)[1] == -1.0


def test_frenar_centrar_salir():
    assert aplicar_tecla(' ', 1.0, 0.5, **LIM) == (0.0, 0.0, False)
    assert aplicar_tecla('c', 1.0, 0.5, **LIM) == (1.0, 0.0, False)
    assert aplicar_tecla('q', 1.0, 0.5, **LIM) == (0.0, 0.0, True)
    assert aplicar_tecla('z', 1.0, 0.5, **LIM) == (1.0, 0.5, False)


def test_sin_restos_de_coma_flotante():
    v = 0.0
    for _ in range(3):
        v, _, _ = aplicar_tecla('w', v, 0.0, **LIM)
    assert v == 0.3


def test_pulsos_como_tabla_de_pendientes():
    assert pulsos_estimados(0.5, 0.0, 0.75, 2.0) == (1550, 1550)
    assert pulsos_estimados(0.5, 1.0, 0.75, 2.0) == (1513, 1588)
    assert pulsos_estimados(0.5, -1.0, 0.75, 2.0) == (1588, 1513)
    assert pulsos_estimados(0.0, 1.0, 0.75, 2.0) == (1500, 1538)
    assert pulsos_estimados(-0.5, 0.0, 0.75, 2.0) == (1500, 1500)


def test_varias_teclas_en_una_lectura():
    assert separar_teclas('ww') == ['w', 'w']
    assert separar_teclas('w\x1b[A\x1b[Ca') == ['w', '\x1b[A', '\x1b[C', 'a']
    # Escape suelto o secuencia desconocida: teclas sueltas que se ignoran.
    assert separar_teclas('\x1b[Z') == ['\x1b', '[', 'Z']
