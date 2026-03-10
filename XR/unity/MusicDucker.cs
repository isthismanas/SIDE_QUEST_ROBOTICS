using UnityEngine;
using System.Collections;

public class MusicDucker : MonoBehaviour
{
    [SerializeField] private AudioSource musicSource;

    [Header("Volumes")]
    [SerializeField] private float normalVolume = 0.25f;
    [SerializeField] private float gameplayVolume = 0.12f;
    [SerializeField] private float boostVolume = 0.38f;

    [Header("Fade")]
    [SerializeField] private float fadeSpeed = 2f;

    [Header("Boost")]
    [SerializeField] private float boostHoldTime = 1.5f;

    private Coroutine fadeRoutine;
    private Coroutine boostRoutine;

    private void Awake()
    {
        if (musicSource == null)
            musicSource = GetComponent<AudioSource>();

        if (musicSource != null)
            musicSource.volume = normalVolume;
    }

    public void LowerMusic()
    {
        StopBoostRoutineIfNeeded();
        StartFade(gameplayVolume);
    }

    public void RestoreMusic()
    {
        StopBoostRoutineIfNeeded();
        StartFade(normalVolume);
    }

    public void BoostSwell()
    {
        StopBoostRoutineIfNeeded();
        boostRoutine = StartCoroutine(BoostSwellRoutine());
    }

    private IEnumerator BoostSwellRoutine()
    {
        yield return FadeToTarget(boostVolume);

        yield return new WaitForSeconds(boostHoldTime);

        yield return FadeToTarget(gameplayVolume);

        boostRoutine = null;
    }

    private void StartFade(float target)
    {
        if (fadeRoutine != null)
            StopCoroutine(fadeRoutine);

        fadeRoutine = StartCoroutine(FadeRoutine(target));
    }

    private IEnumerator FadeRoutine(float target)
    {
        yield return FadeToTarget(target);
        fadeRoutine = null;
    }

    private IEnumerator FadeToTarget(float target)
    {
        if (musicSource == null)
            yield break;

        while (Mathf.Abs(musicSource.volume - target) > 0.01f)
        {
            musicSource.volume = Mathf.Lerp(
                musicSource.volume,
                target,
                Time.deltaTime * fadeSpeed
            );

            yield return null;
        }

        musicSource.volume = target;
    }

    private void StopBoostRoutineIfNeeded()
    {
        if (boostRoutine != null)
        {
            StopCoroutine(boostRoutine);
            boostRoutine = null;
        }
    }
}